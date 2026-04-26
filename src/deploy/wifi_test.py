"""
WiFi 连通性测试 (复用自 lab/ 中的 wifi.py)

用途: 验证笔记本能连上小车 WiFi 并接收数据.
步骤:
  1. 笔记本连接小车 WiFi (WHEELTEC_xxxxx)
  2. 运行本脚本
  3. 按小车复位键
  4. 观察是否能收到数据
"""

import socket

HOST = "192.168.4.1"
PORT = 6390

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print(f"Connecting to {HOST}:{PORT} ...")
s.connect((HOST, PORT))
print("Connected!")

try:
    while True:
        data = s.recv(1024)
        if not data:
            break
        decoded = data.decode("utf-8").strip()
        print(f"Received: {decoded}")
except KeyboardInterrupt:
    print("\nStopped by user")
finally:
    s.close()
