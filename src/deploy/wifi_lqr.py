"""
LQR 基线控制 (复用自 lab/ 中的 wifi3.0.py)

用途: 通过 WiFi 用 LQR 控制小车, 验证通信链路和控制闭环正常工作.
      也作为 RL 效果的对比基线.

步骤:
  1. 烧录修改版固件到小车
  2. 笔记本连接小车 WiFi
  3. 运行本脚本
  4. 按小车复位键, 开启电机
"""

import socket
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))
from src.utils.lqr import lqr_control

HOST = "192.168.4.1"
PORT = 6390


def parse_frame(buffer: str):
    """从 buffer 中提取一帧 {float:float:...} 数据, 返回 (values, remaining_buffer)"""
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
    """打包并发送动作指令: [0xAA][u_L: float32 LE][u_R: float32 LE][checksum]"""
    packet = struct.pack("<Bff", 0xAA, u_L, u_R)
    checksum = sum(packet) & 0xFF
    packet += bytes([checksum])
    sock.send(packet)


def main():
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

            # 解包 13 个 float
            theta_1, theta_dot_1 = values[0], values[1]
            theta_2, theta_dot_2 = values[2], values[3]
            theta_L, theta_R = values[4], values[5]
            theta_L_dot, theta_R_dot = values[6], values[7]

            state = [theta_L, theta_R, theta_L_dot, theta_R_dot,
                     theta_1, theta_dot_1, theta_2, theta_dot_2]
            u_L, u_R = lqr_control(state)

            send_action(s, u_L, u_R)
            print(f"u_L={u_L:.3f}, u_R={u_R:.3f}")

    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        s.close()


if __name__ == "__main__":
    main()
