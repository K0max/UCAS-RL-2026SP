"""
WiFi 通信工具: 帧解析 + buffer drain + 动作打包.

关键设计: 小车以 200Hz 上报状态, Python 控制循环可能只有 ~100Hz.
TCP 内核 buffer 会累积多个帧, 如果按 FIFO 处理就用的是过期状态, 必然失稳.
本模块的 read_latest_state() 会把 socket 里所有可读字节一次性读光, 只解析最后一帧.
"""

import select
import socket
import struct

HOST = "192.168.4.1"
PORT = 6390


def parse_frame(buffer: str):
    """从 buffer 中提取 *最后一帧* {f:f:...:f}, 返回 (values, remaining)."""
    last_end = buffer.rfind("}")
    if last_end == -1:
        return None, buffer
    last_start = buffer.rfind("{", 0, last_end)
    if last_start == -1:
        return None, buffer[last_end + 1:]
    frame = buffer[last_start + 1 : last_end]
    remaining = buffer[last_end + 1:]
    try:
        values = list(map(float, frame.split(":")))
    except ValueError:
        return None, remaining
    if len(values) != 13:
        return None, remaining
    return values, remaining


def drain_socket(sock: socket.socket, prev_buffer: str = "") -> str:
    """非阻塞地把 socket 内核 buffer 里所有数据读光."""
    sock.setblocking(False)
    buffer = prev_buffer
    try:
        while True:
            r, _, _ = select.select([sock], [], [], 0.0)
            if not r:
                break
            chunk = sock.recv(4096)
            if not chunk:
                break
            buffer += chunk.decode("utf-8", errors="ignore")
    finally:
        sock.setblocking(True)
    return buffer


def read_latest_state(sock: socket.socket, timeout: float = 0.1):
    """阻塞等待至少一个字节, 然后 drain 全部, 返回最新一帧的 13 个 float (state).

    返回 None 表示超时无数据.
    """
    sock.settimeout(timeout)
    try:
        first = sock.recv(4096).decode("utf-8", errors="ignore")
    except (socket.timeout, BlockingIOError):
        return None
    buffer = drain_socket(sock, first)
    values, _ = parse_frame(buffer)
    return values


def unpack_state(values):
    """13 个 float -> env-order state [tL, tR, tL_d, tR_d, t1, t1_d, t2, t2_d]."""
    theta_1, theta_dot_1 = values[0], values[1]
    theta_2, theta_dot_2 = values[2], values[3]
    theta_L, theta_R = values[4], values[5]
    theta_L_dot, theta_R_dot = values[6], values[7]
    return [
        theta_L, theta_R, theta_L_dot, theta_R_dot,
        theta_1, theta_dot_1, theta_2, theta_dot_2,
    ]


def send_action(sock: socket.socket, u_L: float, u_R: float):
    """打包动作: [0xAA][u_L f32 LE][u_R f32 LE][checksum]."""
    packet = struct.pack("<Bff", 0xAA, u_L, u_R)
    checksum = sum(packet) & 0xFF
    packet += bytes([checksum])
    sock.sendall(packet)


def connect(host: str = HOST, port: int = PORT) -> socket.socket:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    print(f"Connecting to {host}:{port} ...")
    s.connect((host, port))
    print("Connected!")
    return s
