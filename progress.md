# 项目进度

## 目录结构

```
UCAS-RL-2026SP/
├── pyproject.toml                 # uv 项目配置 & 依赖
├── progress.md                    # ← 你在这里
├── src/
│   ├── env/
│   │   └── balance_car_env.py     # Gym 仿真环境 (线性化状态空间模型)
│   ├── train/
│   │   ├── config.py              # 训练超参数
│   │   └── train.py           # RL 训练脚本 (SAC/PPO/TD3)
│   ├── deploy/
│   │   ├── wifi_test.py           # WiFi 连通性测试
│   │   ├── wifi_lqr.py            # LQR 基线 (WiFi 实车控制)
│   │   └── wifi_rl.py             # RL 模型实车部署
│   ├── imitation/
│   │   ├── collect_expert.py      # 用 LQR 采集专家数据
│   │   └── pretrain_bc.py         # 行为克隆预训练 (可选加分)
│   ├── utils/
│   │   └── lqr.py                 # LQR 控制器 (仿真版 + 实车版)
│   └── validate_env.py            # 验证环境: LQR 能稳住 = 环境没问题
├── data/
│   ├── expert/                    # 专家数据 (LQR 生成)
│   ├── models/                    # 训练好的模型
│   └── logs/                      # TensorBoard 日志
└── firmware/
    └── README.md                  # 固件烧录说明
```

---

## 本次运行完成了什么

### 1. 搭建了完整的项目骨架
- 用 `uv init` 初始化项目, 所有依赖通过 `pyproject.toml` 管理
- 依赖已安装: gymnasium, stable-baselines3, torch, numpy, matplotlib, tensorboard, scipy

### 2. 写好了 Gym 仿真环境 (`src/env/balance_car_env.py`)
- 基于线性化状态空间模型 `x_dot = Ax + Bu`
- A, B 矩阵用二阶倒立摆物理参数构造
- 用 scipy 解 Riccati 方程自动计算匹配的 LQR 增益
- 带非线性重力修正 (大角度下 sin(θ) ≠ θ)
- 支持观测噪声 (sim-to-real 训练用)
- **已验证: matched LQR 19/20 episodes 存活, 环境可用**

### 3. 写好了全部训练和部署代码
- `train.py`: 一键训练 SAC/PPO/TD3, 自动保存 checkpoint + TensorBoard 日志
- `collect_expert.py` + `pretrain_bc.py`: 模仿学习流水线 (加分项)
- `wifi_test.py` / `wifi_lqr.py` / `wifi_rl.py`: 实车部署三件套

### 4. 从已有代码复用了什么
- `lqr.py` 中的实车 K 矩阵直接来自 `wifi3.0.py` / `control.c`
- `wifi_lqr.py` 的通信协议完全复用 `wifi3.0.py` 的帧解析和打包逻辑
- `wifi_test.py` 复用 `wifi.py` 的基础连接代码
- `balance_car_env.py` 中的硬件常量 (轮径、轮距、编码器参数等) 来自 `control.h`

---

## 关键设计决策解释

### 训练数据从哪来？

**不需要预先准备数据。** RL 训练时 agent 自己和仿真环境交互产生数据:

```
agent.predict(state) → action → env.step(action) → next_state, reward
```

这些 (state, action, reward, next_state) 自动存入 replay buffer, SAC/PPO 从中采样训练.
唯一需要"造数据"的场景是可选的模仿学习: 用 LQR 在仿真里跑, 存 (state, action) 对做行为克隆.

### scipy 在干什么？

scipy 做的是**调仿真环境**, 不是训练:

1. 实车的 LQR K 矩阵 (K13=-5492, K14=18921) 是为实车特定动力学设计的
2. 仿真的物理参数 (质量/杆长/惯量) 是估计值, 和实车不完全一致
3. 如果直接把实车 K 矩阵用在仿真里, LQR 全部翻车 → 仿真动力学和实车不匹配
4. 解决方案: 用 `scipy.linalg.solve_continuous_are` 为仿真环境**单独求解匹配的 LQR**
5. 结果: matched LQR 19/20 episode 通过, 说明仿真环境自身是自洽的
6. **代价**: 仿真和实车之间存在 sim-to-real gap, 后续需要缩小

### 嵌入式代码不需要复制进项目

因为我们选的是 **方案 A (PC 端推理)**: Python 跑在电脑上, C 固件跑在 STM32 上.
通信协议 (帧解析/打包) 已经在 `deploy/` 中用 Python 重新实现了.

从嵌入式代码中**已经提取并复用**的信息:
- **通信协议**: `show.c` 的 `APP_Show()` 发送格式, `usart3.c` 的接收解析 → `deploy/wifi_lqr.py`
- **LQR 增益**: `control.c` 的 K11-K28 → `utils/lqr.py`
- **硬件常量**: `control.h` 的轮径/轮距/编码器参数 → `env/balance_car_env.py`
- **控制逻辑**: `control.c` 的 `HAL_GPIO_EXTI_Callback` 中的状态计算方式 → env 的状态定义
- **安全限制**: theta_1 > ±45° 停机 → env 的 `THETA1_LIMIT`

固件本身**不需要修改**, 直接烧录 .hex 即可. 原始代码留在 `lab/` 目录作为参考.

---

## 当前项目进度

| 阶段 | 状态 | 说明 |
|------|------|------|
| 仿真环境 | ✅ 完成 | LQR 验证通过 (19/20) |
| RL 训练代码 | ✅ 完成 | smoke test 通过, 可以直接跑 |
| 模仿学习代码 | ✅ 完成 | collect + pretrain 都写好了 |
| WiFi 部署代码 | ✅ 完成 | 通信协议已实现 |
| 固件烧录指南 | ✅ 完成 | `firmware/README.md`, 含工具/驱动/参考 PDF |
| **RL 训练** | ⏳ 未开始 | 需要你来跑 |
| **实车硬件准备** | ⏳ 未开始 | 需要你动手 (按 `firmware/README.md` 操作) |
| **实车测试** | ⏳ 未开始 | 依赖前两项 |

---

## 你现在该做什么

### 第一步: 跑一次 RL 训练 (纯 Python, 不需要小车)

```bash
cd UCAS-RL-2026SP/

# 先验证环境正常
uv run src/validate_env.py

# 开始训练 (默认 SAC, 50万步, 约需 10-30 分钟)
uv run src/train/train.py

# 或者换算法
uv run src/train/train.py --algo PPO
uv run src/train/train.py --algo TD3

# 查看训练曲线
uv run tensorboard --logdir data/logs
```

### 第二步 (可选加分): 模仿学习预训练

```bash
# 用 LQR 采集专家数据
uv run src/imitation/collect_expert.py

# 行为克隆预训练
uv run src/imitation/pretrain_bc.py
```

### 第三步: 准备实车硬件

详见 `firmware/README.md`, 里面有完整的零基础烧录指南, 包括:
1. 安装 CH9102 USB 驱动
2. 用 mcuisp.exe 烧录固件 (所有工具已在 `firmware/tools/` 中)
3. 把蓝牙模块换成 DT06 WiFi 模块
4. 验证 WiFi 通信

### 第四步: 实车测试

```bash
# 先测连通性
uv run src/deploy/wifi_test.py

# 用 LQR 基线验证通信正常
uv run src/deploy/wifi_lqr.py

# 部署 RL 模型
uv run src/deploy/wifi_rl.py --model data/models/best_model.zip
```

---

## 阻塞项 / 需要注意的问题

### 1. 仿真环境动力学参数不精确 (重要但不阻塞训练)
- 当前环境的物理参数 (质量/杆长/惯量) 是估计值, 和实车有差距
- 这意味着仿真训练的策略直接部署到实车**可能不能直接用**
- **不影响 RL 训练本身** — agent 能在这个仿真里学会平衡, 训练流程是完整的
- **影响实车部署阶段**, 缩小 gap 的方案:
  - 方案 A: 从 MATLAB 仿真 (`inverted_pendulum_on_self_balancing_robot.m`) 提取精确参数
  - 方案 B: 训练时加 domain randomization (随机化物理参数), 提高泛化
  - 方案 C: 先在实车上跑 LQR 收集数据, 用 system identification 拟合模型
  - 方案 D: 直接在实车上做 online RL (WiFi 闭环, 但有摔车风险)

### 2. 实车部署延迟
- WiFi 通信有延迟, Python 推理也有延迟
- 实车控制周期 10ms, WiFi 往返可能 5-20ms
- **解决**: 模型够小 (两层 256 MLP), 推理 <1ms

### 3. MATLAB 不是必需的
- 课程资料提到 MATLAB Simulink + RL Toolbox, 但我们的 Python 方案完全替代
- 如果你有 MATLAB, 可以打开 `inverted_pendulum_on_self_balancing_robot.m` 提取精确参数
