"""
RL 模型实车部署

用法:
  uv run src/deploy/wifi_rl.py                                          # 加载默认 SAC final 模型
  uv run src/deploy/wifi_rl.py --model data/models/balance_ppo_final.zip
  uv run src/deploy/wifi_rl.py --algo PPO
  uv run src/deploy/wifi_rl.py --device cuda                            # 强制 CUDA
  uv run src/deploy/wifi_rl.py --device cpu                             # 强制 CPU

通信协议与 wifi_lqr.py 完全一致, 唯一区别是控制算法从 LQR 换成 RL.

性能优化:
  - 默认 device=auto (有 CUDA 用 CUDA, 否则 CPU)
  - 连接前 warm-up 模型 (前几次推理可能比稳态慢 10x)
  - 用 torch.inference_mode 关闭梯度

注意: 256x256 这种小网络 + batch_size=1, CPU 通常比 CUDA 更快 (避免 H2D/D2H
拷贝). CUDA 主要适合大模型或大 batch. 如果 device=auto 选了 CUDA 但 max_pred
反而变大, 用 --device cpu 强制切换.
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import torch
from stable_baselines3 import SAC, PPO, TD3

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))
from src.deploy.wifi_proto import connect, read_latest_state, send_action, unpack_state

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
ALGO_MAP = {"SAC": SAC, "PPO": PPO, "TD3": TD3}


def resolve_device(requested: str) -> str:
    """`auto` -> cuda if available else cpu. `cuda`/`cpu` 直接返回 (cuda 不可用时 fallback)."""
    cuda_ok = torch.cuda.is_available()
    if requested == "cpu":
        return "cpu"
    if requested == "cuda":
        if not cuda_ok:
            print("[WARN] --device cuda requested but no CUDA available, falling back to CPU.")
            return "cpu"
        return "cuda"
    # auto
    if cuda_ok:
        name = torch.cuda.get_device_name(0)
        print(f"[device] auto -> cuda ({name})")
        return "cuda"
    print("[device] auto -> cpu (no CUDA available)")
    return "cpu"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--algo", default="SAC", choices=list(ALGO_MAP))
    parser.add_argument(
        "--model",
        default=None,
        help="Path to .zip model. Default: data/models/balance_<algo>_final.zip",
    )
    parser.add_argument(
        "--device",
        default="auto",
        choices=["cpu", "cuda", "auto"],
        help=(
            "Inference device. 'auto' uses CUDA if available else CPU. "
            "Note: for small MLPs (~256x256) with batch_size=1, CPU is usually "
            "faster due to H2D/D2H overhead. Switch to --device cpu if max_pred "
            "is high under cuda."
        ),
    )
    parser.add_argument(
        "--action-scale",
        type=float,
        default=20.0,
        help=(
            "Multiply RL action by this factor before sending. "
            "Training uses U_MAX=200, but real firmware expects LQR-magnitude "
            "values (~thousands). Try 10~30 to match LQR scale."
        ),
    )
    parser.add_argument(
        "--action-clip",
        type=float,
        default=8000.0,
        help="Final clamp on |u_L|, |u_R| before sending (safety).",
    )
    args = parser.parse_args()

    model_path = args.model or str(
        REPO_ROOT / "data" / "models" / f"balance_{args.algo.lower()}_final.zip"
    )
    device = resolve_device(args.device)
    print(f"Loading {args.algo} model from {model_path} (device={device}) ...")
    model = ALGO_MAP[args.algo].load(model_path, device=device)

    print("Warming up model (20 dummy inferences) ...")
    dummy = np.zeros(8, dtype=np.float32)
    with torch.inference_mode():
        # First few iters are JIT/cudnn autotune; measure only the last 10.
        for _ in range(10):
            model.predict(dummy, deterministic=True)
        if device == "cuda":
            torch.cuda.synchronize()
        t0 = time.perf_counter()
        for _ in range(10):
            model.predict(dummy, deterministic=True)
        if device == "cuda":
            torch.cuda.synchronize()
        warmup_ms = (time.perf_counter() - t0) * 1000 / 10
    print(f"Avg inference (steady-state): {warmup_ms:.2f} ms/step")
    if warmup_ms > 20:
        print("[WARN] inference >20ms — control may be unstable.")
        if device == "cuda":
            print("       small MLP on CUDA is often slower than CPU; try --device cpu.")

    s = connect()
    print("Waiting for data...")

    n_frames = 0
    n_drops = 0
    t_start = time.perf_counter()
    t_last_print = t_start
    max_predict_ms = 0.0

    try:
        with torch.inference_mode():
            while True:
                values = read_latest_state(s, timeout=0.5)
                if values is None:
                    n_drops += 1
                    print(f"[WARN] no data within 500ms (drops={n_drops})")
                    continue

                state = np.array(unpack_state(values), dtype=np.float32)
                t_pred = time.perf_counter()
                action, _ = model.predict(state, deterministic=True)
                pred_ms = (time.perf_counter() - t_pred) * 1000
                max_predict_ms = max(max_predict_ms, pred_ms)

                if not np.all(np.isfinite(action)):
                    print(f"[ERROR] non-finite action: {action} — sending zero")
                    send_action(s, 0.0, 0.0)
                    continue

                action = np.clip(action * args.action_scale, -args.action_clip, args.action_clip)
                u_L, u_R = float(action[0]), float(action[1])
                send_action(s, u_L, u_R)

                n_frames += 1
                now = time.perf_counter()
                if now - t_last_print > 1.0:
                    rate = n_frames / (now - t_start)
                    print(
                        f"[RL] {rate:5.1f}Hz | u=({u_L:+8.3f},{u_R:+8.3f}) | "
                        f"t1={state[4]:+.3f} t2={state[6]:+.3f} | "
                        f"max_pred={max_predict_ms:.1f}ms drops={n_drops}"
                    )
                    t_last_print = now
                    max_predict_ms = 0.0

    except KeyboardInterrupt:
        print("\nStopped by user")
        send_action(s, 0.0, 0.0)
    finally:
        s.close()


if __name__ == "__main__":
    main()
