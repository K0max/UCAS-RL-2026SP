# UCAS-RL-2026SP

WHEELTEC B585 二阶平衡车的强化学习控制器. 用 RL 策略替代 LQR 控制器, 通过 WiFi 部署到实车.

> 课程作业参考: `tutorial.pdf` (强化学习平衡车资料整理)

## 架构

```
[STM32 固件]                [PC Python]
  传感器 200Hz   ──WiFi──>    策略推理 (RL / LQR)
  电机 PI 控制  <──WiFi──    动作输出 (u_L, u_R)
```

固件只负责传感器读取、状态上报和电机驱动. 控制逻辑 (LQR / RL) 全部在 PC 上跑 Python.

两种运行模式:
- **`Control_mode = 0`** (默认固件): 板载 LQR 自行平衡 + WiFi 上报状态 → 用于**数据采集**
- **`Control_mode = 1`** (修改版固件): 等待 PC 发送控制指令 → 用于 **RL/LQR 实车部署**

## 离线 Pipeline

⚠️ 连上小车 WiFi 后笔记本无法上网. 所有需要网络的步骤先做完:

```bash
git clone https://github.com/k0max/UCAS-RL-2026SP.git
cd UCAS-RL-2026SP
uv sync                          # 下载依赖 (~2GB torch + sb3, 必须有网)
uv run src/validate_env.py       # 验证仿真环境
uv run src/train/train.py --timesteps 300000  # 训练模型 (不需要网)
```

然后再切到小车 WiFi 跑 deploy 脚本.

## 训练

### 方式 1: 仿真训练 (不需要小车)

RL agent 和仿真环境 (`BalanceCarEnv`) 交互, 自动生成训练数据:

```bash
# 三种算法
uv run src/train/train.py --algo SAC     # 推荐, 样本效率高
uv run src/train/train.py --algo PPO
uv run src/train/train.py --algo TD3

# 自定义
uv run src/train/train.py --algo SAC --timesteps 500000 --lr 1e-4

# 加观测噪声 (提高 sim-to-real 鲁棒性)
uv run src/train/train.py --noise 0.05

# TensorBoard 看曲线
uv run tensorboard --logdir data/logs
```

模型保存到 `data/models/balance_<algo>_final.zip`, checkpoint 自动滚动保留最新 2 个.

### 方式 2: 实车数据采集 + 离线训练 (推荐)

让小车以板载 LQR 自行平衡, PC 被动接收真实状态数据:

```bash
# 连接小车 WiFi 后:
uv run src/deploy/collect_real.py --duration 120   # 采集 120 秒
uv run src/deploy/collect_real.py                  # 手动 Ctrl+C 停止
```

输出 `data/expert/real_car_data.npz` (states, actions, next_states).

用途:
- **行为克隆**: 用真实 `(state, action)` 替代仿真数据跑 `pretrain_bc.py`
- **System ID**: 用 `(state, action, next_state)` 拟合真实 A, B 矩阵, 修正仿真参数
- **Offline RL**: 给每个 transition 算 reward, 跑 CQL/IQL

### 模仿学习预训练 (可选加分)

```bash
uv run src/imitation/collect_expert.py    # 用 matched LQR 在仿真里采集专家数据
uv run src/imitation/pretrain_bc.py       # 行为克隆预训练
```

## 实车部署

前置条件: 小车已换 DT06 WiFi 模块, 烧录修改版固件 `firmware/MiniBalance.hex` (`Control_mode = 1`), 详见 `firmware/README.md`.

```bash
# 笔记本连接小车 WiFi (默认密码 12345678)
ip addr | grep 192.168.4    # 确认 IP 网段

# 连通性测试
uv run src/deploy/wifi_test.py

# LQR 基线 (验证通信 + 物理装配)
uv run src/deploy/wifi_lqr.py

# RL 部署
uv run src/deploy/wifi_rl.py
uv run src/deploy/wifi_rl.py --algo PPO
uv run src/deploy/wifi_rl.py --action-scale 30     # 调动作幅度
uv run src/deploy/wifi_rl.py --device cuda          # 有 NVIDIA 卡时
```

### 关于 action-scale

RL 训练时 `U_MAX=200`, 但固件期望 LQR 量级 (几千). `--action-scale` 乘以 RL 输出后再发送, 默认 20.

## 板载部署 (替代 WiFi, 可选进阶)

将训练好的模型固化到 STM32, 完全脱离 PC 运行. 需要用小网络 (F103 无 FPU, 48KB RAM, 256KB ROM):

```bash
# 1. 训练小网络 (64x64, ~19KB float32, 推理 ~1.3ms)
uv run src/train/train.py --net-arch 64 64 --timesteps 300000

# 2. 导出为 C 头文件
uv run src/deploy/export_to_c.py
uv run src/deploy/export_to_c.py --quantize int8   # int8 量化更小更快

# 生成 firmware/rl_model.h, 包含权重数组 + rl_predict() 函数
```

然后在 Keil 工程里:

```c
// control.c
#include "rl_model.h"

// 在 Control_mode == 1 分支中替换 LQR:
float state[8] = {theta_L, theta_R, theta_1, theta_2,
                  theta_L_dot, theta_R_dot, theta_dot_1, theta_dot_2};
float action[2];
rl_predict(state, action);
u_L = action[0];
u_R = action[1];
```

重新编译烧录即可. 板载推理无 WiFi 延迟, 控制频率 = 固件原生 100Hz.

| 网络 | 参数量 | float32 大小 | int8 大小 | STM32 推理 |
|------|--------|-------------|----------|-----------|
| 8→256→256→2 | 68,610 | 268 KB (超 ROM) | 67 KB | ~9ms (太慢) |
| 8→64→64→2 | 4,866 | 19 KB | 5 KB | ~1.3ms |
| 8→32→32→2 | 1,378 | 5.4 KB | 1.3 KB | ~0.4ms |

## 故障排查

### 看控制循环健康度

```
[LQR] 95.3 Hz | u=(+1234.5,+1234.5) | t1=+0.012 t2=-0.008
[RL]  92.1Hz | u=(+3210,+3105) | t1=+0.012 | max_pred=2.1ms drops=0
```

- **Hz** 80~150 正常; <30 WiFi 信号差
- **t1/t2** 平放应接近 0; 越界 → 小车没放正 / IMU 漂移 → 按复位键
- **max_pred** 应 <5ms; >20ms 换 `--device cpu`
- **drops** 应为 0; 持续增长 → WiFi 信号问题

### LQR 不稳 → 物理问题

固件出厂同款 K 矩阵, 不稳排查: 平地放置、摆杆拧紧、电池充满、地面摩擦够大.

### RL 轮子不转 → action-scale 太小

训练时 ±200, 发到固件不够驱动电机. 调大 `--action-scale 20`~`30`.

### RL 数据中断 → 策略太差

5000 步 SAC 几乎是随机策略, 乱抽搐 → 超限触发固件保护 → 停止上报. 训 ≥200k 步再测.

### LQR 稳 + RL 不稳 → sim-to-real gap

核心难点, 缩小方向:
- 加观测噪声: `--noise 0.05`
- Domain randomization: 在 `reset()` 里随机化物理参数
- 行为克隆 warm-start: `collect_expert.py` + `pretrain_bc.py`
- 延长训练: ≥300k 步
- System ID: `collect_real.py` 采集真实数据 → 拟合 A, B → 修正仿真

## 目录结构

```
UCAS-RL-2026SP/
├── pyproject.toml                 # uv 项目配置 & 依赖
├── README.md                      # ← 你在这里
├── tutorial.pdf                   # 课程作业指南
├── src/
│   ├── env/balance_car_env.py     # Gym 仿真环境 (线性化 x_dot = Ax + Bu)
│   ├── train/
│   │   ├── config.py              # TrainConfig 超参
│   │   └── train.py               # SAC / PPO / TD3 训练入口
│   ├── deploy/
│   │   ├── wifi_proto.py          # WiFi 通信工具 (帧解析, buffer drain)
│   │   ├── wifi_test.py           # 连通性测试
│   │   ├── wifi_lqr.py            # LQR 基线 (Control_mode=1)
│   │   ├── wifi_rl.py             # RL 部署 (Control_mode=1)
│   │   ├── collect_real.py        # 实车数据采集 (Control_mode=0)
│   │   └── export_to_c.py         # 模型导出为 C (板载部署)
│   ├── imitation/
│   │   ├── collect_expert.py      # 仿真专家数据
│   │   └── pretrain_bc.py         # 行为克隆预训练
│   ├── utils/lqr.py               # LQR 控制器 (实车 K + 仿真 K)
│   └── validate_env.py            # 仿真环境验证
├── data/
│   ├── expert/                    # 专家数据 (.npz)
│   ├── models/                    # 训练好的模型 (.zip)
│   └── logs/                      # TensorBoard 日志
└── firmware/
    ├── README.md                  # 零基础烧录指南
    ├── MiniBalance.hex            # 预编译固件 (Control_mode=1)
    ├── tools/                     # mcuisp.exe + CH9102 驱动
    └── docs/                      # 烧录截图 + WiFi 配置 PDF
```

## 状态空间与动作空间 (来自 tutorial.pdf)

**状态 (8 维)**:
- `theta_L`, `theta_R`: 左右轮角度 (rad)
- `theta_L_dot`, `theta_R_dot`: 左右轮角速度 (rad/s)
- `theta_1`, `theta_dot_1`: 车身倾角及角速度 (rad, rad/s)
- `theta_2`, `theta_dot_2`: 摆杆倾角及角速度 (rad, rad/s)

**动作 (2 维)**:
- `u_L`, `u_R`: LQR/RL 计算出的左右轮角加速度 (rad/s²)

固件接收到 `(u_L, u_R)` 后, 内部通过 PI 控制器转换为 PWM 驱动电机.

## 设计说明

### 仿真 vs 实车 LQR

固件实车 K 矩阵 (K13=-5492, K14=18921) 为实车特定动力学设计. 仿真物理参数是估计值, 和实车不一致 — 仿真环境用 `scipy.linalg.solve_continuous_are` 为自己求 matched LQR, 验证环境自洽.

### 通信协议

- 小车 → PC: ASCII 帧 `{f1:f2:...:f13}`, 13 个 float (8 状态 + 5 目标)
- PC → 小车: 二进制 `[0xAA][u_L: f32 LE][u_R: f32 LE][checksum]`, 共 10 字节
