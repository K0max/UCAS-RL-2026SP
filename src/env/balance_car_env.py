"""
二阶倒立摆平衡车 Gymnasium 环境

基于线性化状态空间模型 x_dot = Ax + Bu.
A, B 矩阵参数来自二阶倒立摆的拉格朗日动力学简化.
LQR 增益通过 scipy 求解 Riccati 方程得到, 保证与环境匹配.

状态 (8维): [theta_L, theta_R, theta_L_dot, theta_R_dot,
              theta_1, theta_dot_1, theta_2, theta_dot_2]
动作 (2维): [u_L, u_R] 左右轮角加速度 (rad/s^2)

TODO: 对准 MATLAB 仿真参数以缩小 sim-to-real gap.
"""

import gymnasium as gym
import numpy as np
from gymnasium import spaces
from scipy import linalg


# ===== 实车硬件常量 (来自 control.h) =====
WHEEL_DIAMETER_MM = 67.0
WHEEL_RADIUS_M = WHEEL_DIAMETER_MM / 2 / 1000      # 0.0335 m
WHEEL_PERIMETER_MM = 210.4867
WHEEL_SPACING_MM = 160.0                            # 轮距
ENCODER_PRECISION = 500.0                           # 编码器线数
ENCODER_MULTIPLES = 4.0                             # 四倍频
REDUCTION_RATIO = 30.0                              # 减速比
CONTROL_FREQUENCY_HZ = 200.0                        # 传感器采样 (5ms), 控制周期 10ms


def build_AB():
    """
    构造线性化 A, B 矩阵.

    状态向量 (K-matrix ordering, 用于 LQR):
      x_k = [theta_L, theta_R, theta_1, theta_2,
             theta_L_dot, theta_R_dot, theta_dot_1, theta_dot_2]

    注意: env 的 state 用的是另一种顺序 (把角度和角速度交替排), 需要转换.

    物理参数说明:
      这些参数是近似值. 要缩小 sim-to-real gap, 可以:
      - 从 MATLAB 仿真 (inverted_pendulum_on_self_balancing_robot.m) 提取
      - 或在实车上做 system identification
    """
    g = 9.81
    r = WHEEL_RADIUS_M
    # 车身: 等效摆长, 重心到轮轴距离
    l1 = 0.15    # m (估计值, 需校准)
    I1 = 0.02    # kg*m^2 (含电机/电池等, 估计值)
    m1 = 1.5     # kg (估计值)
    # 摆杆
    l2 = 0.12    # m (估计值)
    I2 = 0.005   # kg*m^2 (估计值)
    m2 = 0.3     # kg (估计值)

    # 线性化后: theta_ddot = (mgl/I)*theta - (m*r*l/I)*u_avg (+ 耦合项)
    a_body = m1 * g * l1 / I1        # 车身不稳定极点 ~110
    b_body = -m1 * r * l1 / I1       # 控制耦合 ~-3.8

    a_pend = m2 * g * l2 / I2        # 摆杆不稳定极点 ~70
    b_pend = -m2 * r * l2 / I2 * 0.3 # 摆杆间接耦合 (弱)
    c_cross = 0.5                     # 车身角加速度对摆杆的耦合

    # K-order state: [tL, tR, t1, t2, tL_d, tR_d, t1_d, t2_d]
    A = np.zeros((8, 8))
    # 位置导数 = 速度
    A[0, 4] = 1.0  # d(theta_L)/dt = theta_L_dot
    A[1, 5] = 1.0
    A[2, 6] = 1.0  # d(theta_1)/dt = theta_dot_1
    A[3, 7] = 1.0
    # 角加速度
    A[6, 2] = a_body   # theta_1 gravity (unstable)
    A[7, 3] = a_pend   # theta_2 gravity (unstable)
    A[7, 2] = c_cross   # cross coupling: body tilt -> pendulum
    # 阻尼
    A[6, 6] = -0.5     # 车身阻尼 (轮子摩擦)
    A[7, 7] = -0.3     # 摆杆阻尼

    B = np.zeros((8, 2))
    B[4, 0] = 1.0   # u_L -> theta_L_dot_dot
    B[5, 1] = 1.0   # u_R -> theta_R_dot_dot
    B[6, 0] = b_body / 2; B[6, 1] = b_body / 2  # avg_u -> body
    B[7, 0] = b_pend / 2; B[7, 1] = b_pend / 2  # avg_u -> pend (weak)

    return A, B


def compute_lqr_gain(A, B, Q=None, R=None):
    """通过求解连续时间 Riccati 方程计算 LQR 增益 K."""
    if Q is None:
        Q = np.diag([1.0, 1.0, 500.0, 1000.0, 1.0, 1.0, 10.0, 50.0])
    if R is None:
        R = np.diag([0.01, 0.01])

    P = linalg.solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ B.T @ P
    return K


# 环境 state 和 K-matrix state 的映射
# env: [theta_L, theta_R, theta_L_dot, theta_R_dot, theta_1, theta_dot_1, theta_2, theta_dot_2]
# K:   [theta_L, theta_R, theta_1, theta_2, theta_L_dot, theta_R_dot, theta_dot_1, theta_dot_2]
_ENV_TO_K = [0, 1, 4, 6, 2, 3, 5, 7]
_K_TO_ENV = [0, 1, 4, 5, 2, 6, 3, 7]


class BalanceCarEnv(gym.Env):
    metadata = {"render_modes": ["human"], "render_fps": 100}

    DT = 0.01
    SUBSTEPS = 5
    MAX_STEPS = 1000
    THETA1_LIMIT = np.pi / 4
    THETA2_LIMIT = np.pi / 3
    U_MAX = 200.0

    def __init__(self, render_mode=None, noise_std=0.0):
        super().__init__()
        self.render_mode = render_mode
        self.noise_std = noise_std

        self.A, self.B = build_AB()
        self._K_lqr = compute_lqr_gain(self.A, self.B)

        # Verify closed-loop is stable
        CL = self.A - self.B @ self._K_lqr
        eigs = np.linalg.eigvals(CL)
        assert np.all(eigs.real < 0), f"Closed-loop unstable! eigs={eigs}"

        high_obs = np.array([
            50.0, 50.0, 50.0, 50.0,
            self.THETA1_LIMIT, 20.0,
            self.THETA2_LIMIT, 20.0,
        ], dtype=np.float32)

        self.observation_space = spaces.Box(-high_obs, high_obs, dtype=np.float32)
        self.action_space = spaces.Box(
            low=-self.U_MAX, high=self.U_MAX, shape=(2,), dtype=np.float32
        )
        self.state: np.ndarray = np.zeros(8, dtype=np.float32)
        self.step_count = 0

    @property
    def matched_lqr_gain(self):
        """与本环境动力学匹配的 LQR 增益 (K-order state)."""
        return self._K_lqr

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        super().reset(seed=seed)
        s = np.zeros(8, dtype=np.float32)
        s[0:4] = self.np_random.uniform(-0.01, 0.01, size=4)
        s[4:8] = self.np_random.uniform(-0.01, 0.01, size=4)
        self.state = s
        self.step_count = 0
        return self._get_obs(), {}

    def _env_to_k(self, state_env):
        return state_env[_ENV_TO_K]

    def _k_to_env(self, state_k):
        return state_k[_K_TO_ENV]

    def step(self, action):
        action = np.clip(action, -self.U_MAX, self.U_MAX).astype(np.float32)

        # Convert env-order state to K-order for dynamics
        x_k = self._env_to_k(self.state).astype(np.float64)
        u = action.astype(np.float64)

        sub_dt = self.DT / self.SUBSTEPS
        for _ in range(self.SUBSTEPS):
            # Semi-implicit Euler on linearized dynamics
            x_dot = self.A @ x_k + self.B @ u

            # Add nonlinear gravity correction for large angles
            theta_1 = x_k[2]
            theta_2 = x_k[3]
            a_body = self.A[6, 2]
            a_pend = self.A[7, 3]
            x_dot[6] += a_body * (np.sin(theta_1) - theta_1)  # sin correction
            x_dot[7] += a_pend * (np.sin(theta_2) - theta_2)

            x_k += x_dot * sub_dt

        self.state = self._k_to_env(x_k).astype(np.float32)
        self.step_count += 1

        theta_1 = float(self.state[4])
        theta_2 = float(self.state[6])
        terminated = bool(abs(theta_1) > self.THETA1_LIMIT or abs(theta_2) > self.THETA2_LIMIT)
        truncated = self.step_count >= self.MAX_STEPS

        reward = self._compute_reward(self.state, action, terminated)
        return self._get_obs(), reward, terminated, truncated, {}

    def _compute_reward(self, state, action, terminated):
        _, _, _, _, theta_1, theta_dot_1, theta_2, theta_dot_2 = state
        theta_L_dot, theta_R_dot = state[2], state[3]

        if terminated:
            return -100.0

        angle_cost = 10.0 * theta_1 ** 2 + 20.0 * theta_2 ** 2
        vel_cost = 0.1 * theta_dot_1 ** 2 + 0.1 * theta_dot_2 ** 2
        wheel_cost = 0.01 * (theta_L_dot ** 2 + theta_R_dot ** 2)
        action_cost = 0.001 * (action[0] ** 2 + action[1] ** 2)
        alive_bonus = 1.0

        return alive_bonus - angle_cost - vel_cost - wheel_cost - action_cost

    def _get_obs(self):
        obs = self.state.copy()
        if self.noise_std > 0:
            obs += self.np_random.normal(0, self.noise_std, size=obs.shape).astype(np.float32)
        return obs
