"""
LQR 基线控制 (复用自参考代码中的 wifi3.0.py)

用途: 通过 WiFi 用 LQR 控制小车, 验证通信链路和控制闭环正常工作.
      也作为 RL 效果的对比基线.

步骤:
  1. 烧录修改版固件到小车
  2. 笔记本连接小车 WiFi
  3. 运行本脚本
  4. 按小车复位键, 开启电机

控制延迟优化:
  - 启用 TCP_NODELAY
  - 每次循环 drain socket buffer, 只用最新一帧 (避免用过期状态)
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))
from src.utils.lqr import lqr_control
from src.deploy.wifi_proto import connect, read_latest_state, send_action, unpack_state


def main():
    s = connect()
    print("Waiting for data...")

    n_frames = 0
    t_start = time.perf_counter()
    t_last_print = t_start

    try:
        while True:
            values = read_latest_state(s, timeout=0.5)
            if values is None:
                print("[WARN] no data within 500ms")
                continue

            state = unpack_state(values)
            u_L, u_R = lqr_control(state)
            send_action(s, u_L, u_R)

            n_frames += 1
            now = time.perf_counter()
            if now - t_last_print > 1.0:
                rate = n_frames / (now - t_start)
                t1 = state[4]
                t2 = state[6]
                print(
                    f"[LQR] {rate:5.1f} Hz | u=({u_L:+8.1f},{u_R:+8.1f}) | t1={t1:+.3f} t2={t2:+.3f}"
                )
                t_last_print = now

    except KeyboardInterrupt:
        print("\nStopped by user")
        send_action(s, 0.0, 0.0)
    finally:
        s.close()


if __name__ == "__main__":
    main()
