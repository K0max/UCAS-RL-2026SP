"""
LQR 控制器 (Python 版)

提供两种模式:
  1. matched_lqr_control(): 使用与仿真环境匹配的 LQR 增益 (仿真用)
  2. real_car_lqr_control(): 使用实车固件中的 K 矩阵 (部署对比用)
"""

import numpy as np

# ========== 实车 K 矩阵 (来自 wifi3.0.py / control.c) ==========
# K-matrix state order: [theta_L, theta_R, theta_1, theta_2,
#                        theta_L_dot, theta_R_dot, theta_dot_1, theta_dot_2]
K_REAL = np.array([
    [81.2695, -10.0616, -5492.4061, 18921.7098, 100.3633, 8.0376, 447.3084, 2962.7738],
    [-10.0616, 81.2695, -5492.4061, 18921.7098, 8.0376, 100.3633, 447.3084, 2962.7738],
])

# env state -> K-matrix state 的索引映射
# env: [theta_L, theta_R, theta_L_dot, theta_R_dot, theta_1, theta_dot_1, theta_2, theta_dot_2]
# K:   [theta_L, theta_R, theta_1, theta_2, theta_L_dot, theta_R_dot, theta_dot_1, theta_dot_2]
_ENV_TO_K = [0, 1, 4, 6, 2, 3, 5, 7]


def _env_state_to_k_state(state_env, target=None):
    """将 env-order state 转为 K-order error state."""
    if target is None:
        target = np.zeros(5)

    theta_L, theta_R, theta_L_dot, theta_R_dot, \
        theta_1, theta_dot_1, theta_2, theta_dot_2 = state_env

    t_theta_L, t_theta_R, t_theta_L_dot, t_theta_R_dot, t_theta_1 = target

    return np.array([
        theta_L - t_theta_L,
        theta_R - t_theta_R,
        theta_1 - t_theta_1,
        theta_2,
        theta_L_dot - t_theta_L_dot,
        theta_R_dot - t_theta_R_dot,
        theta_dot_1,
        theta_dot_2,
    ])


def real_car_lqr_control(state, target=None):
    """
    使用实车 K 矩阵的 LQR 控制器 (用于 WiFi 部署).

    参数:
        state: 长度 8 的数组 (env order)
        target: 长度 5 的数组 [Target_theta_L, ..., Target_theta_1] (可选)
    返回:
        (u_L, u_R)
    """
    x = _env_state_to_k_state(state, target)
    u = -K_REAL @ x
    return float(u[0]), float(u[1])


def matched_lqr_control(state, K_matched, target=None):
    """
    使用与仿真环境匹配的 LQR 控制器 (用于仿真验证).

    参数:
        state: 长度 8 的数组 (env order)
        K_matched: 从 env.matched_lqr_gain 获取的 K 矩阵
        target: 长度 5 的数组 (可选)
    返回:
        (u_L, u_R)
    """
    x = _env_state_to_k_state(state, target)
    u = -K_matched @ x
    return float(u[0]), float(u[1])


# 向后兼容
lqr_control = real_car_lqr_control
