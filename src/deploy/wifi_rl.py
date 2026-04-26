"""
RL 模型实车部署

用法:
  uv run src/deploy/wifi_rl.py                                     # 加载默认模型
  uv run src/deploy/wifi_rl.py --model data/models/best_model.zip  # 指定模型

通信协议与 wifi_lqr.py 完全一致, 唯一区别是控制算法从 LQR 换成 RL.
"""

import argparse
import socket
import struct
import sys
from pathlib import Path

import numpy as np
from stable_baselines3 import SAC

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
HOST = "192.168.4.1"
PORT = 6390


def parse_frame(buffer: str):
    start = buffer.find("{")
    end = buffer.find("}", start)
    if start == -1 or end == -1:
        return None, buffer
    frame = buffer[start + 1 : end]
    remaining = buffer[end + 1 :]
    try:
        values = list(map(float, frame.split(":")))
    except ValueError:
        return None, remaining
    if len(values) != 13:
        return None, remaining
    return values, remaining


def send_action(sock, u_L: float, u_R: float):
    packet = struct.pack("<Bff", 0xAA, u_L, u_R)
    checksum = sum(packet) & 0xFF
    packet += bytes([checksum])
    sock.send(packet)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model",
        default=str(REPO_ROOT / "data" / "models" / "best_model.zip"),
        help="Path to trained RL model (.zip)",
    )
    args = parser.parse_args()

    print(f"Loading model from {args.model} ...")
    model = SAC.load(args.model)
    print("Model loaded!")

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print(f"Connecting to {HOST}:{PORT} ...")
    s.connect((HOST, PORT))
    print("Connected! Waiting for data...")

    buffer = ""
    try:
        while True:
            data = s.recv(128).decode()
            buffer += data

            values, buffer = parse_frame(buffer)
            if values is None:
                continue

            theta_1, theta_dot_1 = values[0], values[1]
            theta_2, theta_dot_2 = values[2], values[3]
            theta_L, theta_R = values[4], values[5]
            theta_L_dot, theta_R_dot = values[6], values[7]

            state = np.array([
                theta_L, theta_R, theta_L_dot, theta_R_dot,
                theta_1, theta_dot_1, theta_2, theta_dot_2,
            ], dtype=np.float32)

            action, _ = model.predict(state, deterministic=True)
            u_L, u_R = float(action[0]), float(action[1])

            send_action(s, u_L, u_R)
            print(f"[RL] u_L={u_L:.3f}, u_R={u_R:.3f} | theta_1={theta_1:.4f} theta_2={theta_2:.4f}")

    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        s.close()


if __name__ == "__main__":
    main()
