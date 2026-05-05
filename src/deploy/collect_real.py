"""
从实车采集 (state, action, next_state) 训练数据.

前提: 固件以 Control_mode = 0 (本地 LQR) 运行, 小车自行保持平衡,
WiFi 以低频发送状态帧. PC 被动接收, 不发任何指令.

输出: data/expert/real_car_data.npz
  - states:      (N, 8)  当前状态 (env order)
  - actions:     (N, 2)  LQR 动作 (由 K_REAL @ state 计算)
  - next_states: (N, 8)  下一帧状态

用法:
  uv run src/deploy/collect_real.py                # 采集到 Ctrl+C
  uv run src/deploy/collect_real.py --duration 60  # 采集 60 秒
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))
from src.deploy.wifi_proto import connect, parse_frame, unpack_state
from src.utils.lqr import real_car_lqr_control

REPO_ROOT = Path(__file__).resolve().parent.parent.parent


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=0,
                        help="Collect for N seconds (0 = until Ctrl+C)")
    parser.add_argument("--output", default=str(REPO_ROOT / "data" / "expert" / "real_car_data.npz"))
    args = parser.parse_args()

    s = connect()
    print("Passively collecting data (NOT sending any commands) ...")
    print("Press Ctrl+C to stop and save.\n")

    frames = []
    n_received = 0
    t_start = time.perf_counter()
    t_last_print = t_start
    buffer = ""

    try:
        while True:
            s.settimeout(1.0)
            try:
                chunk = s.recv(4096).decode("utf-8", errors="ignore")
            except Exception:
                continue
            buffer += chunk

            while True:
                values, buffer = parse_frame(buffer)
                if values is None:
                    break
                frames.append(unpack_state(values))
                n_received += 1

            now = time.perf_counter()
            if now - t_last_print > 2.0:
                elapsed = now - t_start
                hz = n_received / elapsed if elapsed > 0 else 0
                print(f"  [{elapsed:.0f}s] frames={n_received}  rate={hz:.1f} Hz")
                t_last_print = now

            if args.duration > 0 and (now - t_start) > args.duration:
                break

    except KeyboardInterrupt:
        print("\n\nStopped by user.")
    finally:
        s.close()

    if len(frames) < 2:
        print("Not enough data to save (need at least 2 frames).")
        return

    all_states = np.array(frames, dtype=np.float32)
    states = all_states[:-1]
    next_states = all_states[1:]

    # 用 K_REAL 从状态反算 LQR 动作 (固件用的是同一个 K 矩阵)
    actions = np.zeros((len(states), 2), dtype=np.float32)
    for i, st in enumerate(states):
        u_L, u_R = real_car_lqr_control(st)
        actions[i] = [u_L, u_R]

    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(str(out_path), states=states, actions=actions, next_states=next_states)

    elapsed = time.perf_counter() - t_start
    print(f"\nSaved {len(states)} transitions to {out_path}")
    print(f"  Duration: {elapsed:.1f}s")
    print(f"  Effective rate: {len(states)/elapsed:.1f} Hz")
    print(f"  State range: theta_1=[{states[:,4].min():.3f}, {states[:,4].max():.3f}]  "
          f"theta_2=[{states[:,6].min():.3f}, {states[:,6].max():.3f}]")


if __name__ == "__main__":
    main()
