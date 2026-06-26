# Nav2 MPPI Controller — OptimizerSettings 参数说明

> 文件位置：`nav2_mppi_controller/include/nav2_mppi_controller/models/optimizer_settings.hpp`

---

## 1. 控制约束参数

### `base_constraints` / `constraints` (`ControlConstraints`)

两组控制约束，`base_constraints` 是基础/标称约束，`constraints` 是运行时可覆盖的约束。

| 字段 | 含义 |
|------|------|
| `vx_max` | 线速度 x 方向上限（m/s） |
| `vx_min` | 线速度 x 方向下限（m/s） |
| `vy` | y 方向线速度限制（m/s，差速模型通常为 0） |
| `wz` | 角速度 z 轴限制（rad/s） |
| `ax_max` | x 方向线加速度上限（m/s²） |
| `ax_min` | x 方向线加速度下限（m/s²） |
| `ay_max` | y 方向线加速度上限（m/s²） |
| `ay_min` | y 方向线加速度下限（m/s²） |
| `az_max` | z 轴角加速度上限（rad/s²） |

---

## 2. 采样噪声参数

### `sampling_std` (`SamplingStd`)

MPPI 的核心机制——在控制序列上叠加**高斯噪声**来探索轨迹空间。

| 字段 | 含义 |
|------|------|
| `vx` | 线速度 vx 的噪声标准差 — 越大，速度探索范围越广 |
| `vy` | 横向速度 vy 的噪声标准差 |
| `wz` | 角速度 wz 的噪声标准差 — 越大，转向探索范围越广 |

---

## 3. 模型与延迟参数

| 参数 | 含义 |
|------|------|
| `model_dt` | 模型离散化时间步长（s），即轨迹预测每一步的时间间隔 |
| `model_delay_vx` | vx 控制指令的执行延迟（s），模拟指令下发到实际响应的滞后 |
| `model_delay_vy` | vy 控制指令的执行延迟（s） |
| `model_delay_wz` | wz 角速度指令的执行延迟（s） |

---

## 4. MPPI 核心超参数

| 参数 | 含义 |
|------|------|
| `temperature` | **温度参数 λ**（>0）。控制代价加权时的"软度"：越小越趋近贪心（选最低代价轨迹），越大越平滑（熵更大，探索性更强） |
| `gamma` | **折扣因子**，控制未来时刻代价的衰减权重。接近 0 时只关心近期代价，接近 1 时远期代价同样重要 |
| `batch_size` | 每次迭代采样的轨迹条数 K。K 越大探索越充分，但计算量线性增长 |
| `time_steps` | 每条轨迹的预测步数 T。总预测时长为 `T × model_dt` |
| `iteration_count` | MPPI 优化的迭代次数。每轮控制周期内重复"采样 → 加权 → 更新"的轮数 |

---

## 5. 控制器执行参数

| 参数 | 含义 |
|------|------|
| `controller_period` | 控制器运行周期（s），即多久输出一次 `cmd_vel` |

---

## 6. 其他设置

| 参数 | 含义 |
|------|------|
| `shift_control_sequence` | 是否将上一轮的最优控制序列**向前平移一步**作为下一轮的初始序列（warm-start），可提升收敛速度和时序一致性 |
| `retry_attempt_limit` | 如果生成的轨迹全部无效（碰撞），允许重试的最大次数 |
| `open_loop` | 开环模式：`true` 时不使用状态反馈，仅靠模型推演（通常用于仿真/调试） |
| `sgf_order` | Savitzky-Golay Filter 阶数，用于对控制序列做平滑滤波（默认 2 阶），抑制高频抖动 |

---

## MPPI 算法流程概要

```
1. 获取当前状态 + 参考路径
2. 在 time_steps 时域上，采样 batch_size 条带高斯噪声的控制序列
   （噪声标准差由 sampling_std 控制）
3. 用 model_dt 步长推演每条轨迹，计算总代价
4. 用 temperature 对代价做 softmin 加权，得到最优控制序列
5. 重复 iteration_count 轮（每轮用上次结果 warm-start）
6. 输出第一个控制量作为 cmd_vel
7. 如果 shift_control_sequence=true，将序列平移一步供下一周期使用
```

---

# MPPI State 数据结构

> 文件位置：`nav2_mppi_controller/include/nav2_mppi_controller/models/state.hpp`

`State` 是 MPPI 控制器中贯穿整个优化过程的**核心数据容器**。

---

## 1. 状态速度矩阵（经运动模型推演后的速度）

```cpp
Eigen::ArrayXXf vx;  // (batch_size × time_steps)
Eigen::ArrayXXf vy;
Eigen::ArrayXXf wz;
```

- 每一**行** = 一条采样轨迹（共 `batch_size` 条）
- 每一**列** = 该轨迹在某个时间步（共 `time_steps` 步）的速度
- 第 0 列初始化为当前机器人速度（来自 `speed`），后续列由控制量经运动模型递推得到
- 被各 Critic（代价评估器）用来评估轨迹优劣（碰撞检测、约束违反检查等）

---

## 2. 控制量矩阵（带噪声的候选控制序列）

```cpp
Eigen::ArrayXXf cvx;  // (batch_size × time_steps)
Eigen::ArrayXXf cvy;
Eigen::ArrayXXf cwz;
```

**生成公式**：

```
cvx = 基准控制序列 + N(0, sampling_std.vx²)
cvy = 基准控制序列 + N(0, sampling_std.vy²)
cwz = 基准控制序列 + N(0, sampling_std.wz²)
```

- 基准控制序列来自上一轮优化结果（或全零初始）
- 叠加高斯噪声实现 MPPI 的随机探索
- 这组带噪控制量输入运动模型 → 产出上方的 `vx/vy/wz`

---

## 3. 机器人当前状态

```cpp
geometry_msgs::msg::PoseStamped pose;  // 当前位姿 (x, y, yaw)
geometry_msgs::msg::Twist speed;        // 当前速度 (linear.x/y/z, angular.x/y/z)
float local_path_length;                // 局部参考路径长度
```

| 字段 | 来源 | 用途 |
|------|------|------|
| `pose` | 定位系统 (amcl/slam) | 轨迹推演的起点坐标和朝向 |
| `speed` | 里程计 (odom) | 初始化 `vx/vy/wz` 矩阵第 0 列 |
| `local_path_length` | 全局规划器 | 供 Critic 做距离归一化等 |

---

## 4. 关键区分：`cvx` vs `vx`

| | `cvx / cvy / cwz` | `vx / vy / wz` |
|---|---|---|
| **含义** | 带噪声的**控制输入**序列 | 经运动模型推演后的**状态速度** |
| **类比** | "油门/方向盘指令" | "执行后的实际速度" |
| **来源** | 基准控制 + 高斯噪声采样 | 由 `cvx` 经运动模型递推得到 |
| **使用者** | 运动模型（输入） | Critic（代价评估） |

---

## 5. State 在 MPPI 中的数据流

```
┌─────────────────────────────────────────────────────┐
│  1. 读取传感器                                        │
│     state.pose  ← 定位                                │
│     state.speed ← 里程计                              │
├─────────────────────────────────────────────────────┤
│  2. 生成噪声控制量 (NoiseGenerator)                    │
│     state.cvx ← 基准序列 + N(0, sampling_std.vx²)    │
│     state.cvy ← 基准序列 + N(0, sampling_std.vy²)    │
│     state.cwz ← 基准序列 + N(0, sampling_std.wz²)    │
├─────────────────────────────────────────────────────┤
│  3. 运动模型推演 (MotionModel::predict)               │
│     state.vx[:,0] ← state.speed                      │
│     state.vx[:,t] ← f(cvx[:,t-1], 上一步状态)        │
│     ...同理 vy, wz                                    │
├─────────────────────────────────────────────────────┤
│  4. 代价评估 (各 Critic)                              │
│     Critic 读取 state.vx/vy/wz ──► 计算代价          │
│     例：ConstraintCritic 检查 vx 是否超 vx_max       │
├─────────────────────────────────────────────────────┤
│  5. 加权平均 → 输出最优控制量                          │
└─────────────────────────────────────────────────────┘
```

---

## 6. `reset()` 方法

```cpp
void reset(unsigned int batch_size, unsigned int time_steps)
```

每轮优化开始时调用，将所有速度矩阵 `vx/vy/wz` 和 `cvx/cvy/cwz` 清零并调整尺寸为 `(batch_size × time_steps)`。`pose`、`speed`、`local_path_length` 由外部单独更新。

---

# MPPI Optimizer 核心引擎

> 文件位置：`nav2_mppi_controller/include/nav2_mppi_controller/optimizer.hpp`
> 实现文件：`nav2_mppi_controller/src/optimizer.cpp`

`Optimizer` 是 MPPI 控制器的核心引擎，负责将传感器输入转化为 `cmd_vel` 输出。

---

## 1. 公有接口 — 对外 API

### 生命周期

| 方法 | 作用 |
|------|------|
| `initialize()` | 加载参数、初始化运动模型/噪声生成器/CriticManager/轨迹验证器 |
| `shutdown()` | 停止噪声生成器 |
| `reset()` | 将所有状态、控制序列、代价矩阵归零 |

### 主入口：`evalControl()`

每一轮控制周期的入口函数，被 `nav2_controller` 调用：

```
evalControl(pose, speed, plan, goal, checker)
  ├─ prepare()          ← 填充 state_.pose/speed/path/goal
  ├─ optimize()         ← MPPI 核心迭代（可能重试）
  ├─ 轨迹验证           ← validator 检查最优轨迹合法性
  ├─ getControlFromSequenceAsTwist()  ← 输出 cmd_vel
  └─ shiftControlSequence()  ← warm-start 准备
```

### 查询接口

| 方法 | 返回 |
|------|------|
| `getGeneratedTrajectories()` | 所有采样轨迹（可视化用） |
| `getOptimizedTrajectory()` | 最优轨迹的 x/y/yaw 序列 |
| `getOptimalControlSequence()` | 最优控制序列 vx/wz(/vy) |
| `getCosts()` | 每条轨迹的总代价 |
| `getCriticCosts()` | 每个 Critic 的代价分解 |
| `getCollisionFlags()` | 每条轨迹是否碰撞 |
| `getSettings()` | 当前优化器参数 |

### 速度限制

```cpp
setSpeedLimit(limit, percentage);   // 动态限速（来自 costmap 的 speed_limit 区域）
isSpeedLimitActive();               // 判断当前是否有动态限速
```

当限速激活时，`constraints` ≠ `base_constraints`，并拒绝动态修改运动学参数。

---

## 2. 核心流程：`prepare()` → `optimize()` → 输出

```
evalControl()
│
├─ prepare()   ← 预测速度、设置 pose/path/goal、costs 清零
│
└─ do {                          ← retry 循环
│     optimize()
│       for iteration_count 次:
│         ├─ generateNoisedTrajectories()
│         │   ├─ applyControlSequenceInterIterationConstraints()  ← 保证 t=0 动力学可行
│         │   ├─ noise_generator_.setNoisedControls()  ← cvx = 基准 + 噪声
│         │   ├─ noise_generator_.generateNextNoises() ← 生成下一轮噪声
│         │   ├─ updateStateVelocities()
│         │   │   ├─ vx[:,0] = speed            ← 初始化
│         │   │   └─ motion_model_->predict()   ← 递推 vx 矩阵
│         │   └─ integrateStateVelocities()     ← 速度 → 位姿轨迹
│         │
│         ├─ critic_manager_.evalTrajectoriesScores()  ← 所有 Critic 打分
│         │
│         └─ updateControlSequence()
│             ├─ 计算 MPPI 代价（状态代价 + 控制正则项）
│             ├─ Softmax 加权平均 → 最优控制序列
│             ├─ Savitzky-Golay 平滑滤波
│             └─ applyControlSequenceConstraints()  ← 硬约束限幅
│
│     └─ trajectory_validator_->validateTrajectory()
│         ├─ SUCCESS     → 退出
│         ├─ SOFT_RESET  → 重试（reset + retry）
│         └─ FAILURE     → 抛异常
│   } while (fallback(...))
│
├─ getControlFromSequenceAsTwist()
│   取 control_sequence[offset]
│   若 shift_control_sequence=true，则 offset=1（用 vx[1]）
│
└─ shiftControlSequence()
    控制序列整体左移一位，末尾复制，供下一轮 warm-start
```

---

## 3. 关键方法详解

### 3.1 `updateControlSequence()` — MPPI 核心公式

```cpp
// 1. 状态代价 + 控制正则化项
//    bounded_noises = cvx - 基准控制 = 纯噪声分量
//    costs += gamma / σ² × (noise ⊙ control).逐行内积
costs_ += (gamma / σ²) * (bounded_noises ⊙ control_sequence).rowwise().sum()

// 2. Softmax 归一化
weights = exp(-(costs - min(costs)) / temperature)
weights /= sum(weights)

// 3. 加权平均得最优控制序列
control_sequence = cvx^T × weights
```

对应 MPPI 理论公式：

$$u^* = \frac{\sum_{k=1}^{K} \exp\left(-\frac{1}{\lambda} S_k\right) \cdot \epsilon_k}{\sum_{k=1}^{K} \exp\left(-\frac{1}{\lambda} S_k\right)}$$

其中 $S_k = C_{state} + \gamma \sum_t u_t \cdot \epsilon_t^k$。

### 3.2 `integrateStateVelocities()` — 速度积分为位姿

对每条轨迹，将 `vx/vy/wz` → `x/y/yaw`：

```
yaw[t] = yaw[t-1] + wz[t] × dt
x[t]   = x[t-1]   + (vx[t]×cos(yaw[t-1]) - vy[t]×sin(yaw[t-1])) × dt
y[t]   = y[t-1]   + (vx[t]×sin(yaw[t-1]) + vy[t]×cos(yaw[t-1])) × dt
```

注意 yaw 用的是**上一时刻的值**来计算位移（半隐式欧拉积分）。

### 3.3 `prepare()` 中的速度预测

```cpp
// 补偿从传感器测量到控制生效之间的延迟
state_.speed.linear.x = clamp(
    last_command_vel_.x,                // 上一帧发出的指令
    当前速度 + dt × ax_min,             // 最慢可能
    当前速度 + dt × ax_max              // 最快可能
);
```

这解决了传感器延迟问题：传感器读到 0.3m/s，但上帧已经发了 0.5m/s 指令，机器人实际已加速。用上一帧指令 clamp 到物理可行范围做预测，比直接用传感器值更准。

---

## 4. 成员变量一览

| 变量 | 类型 | 作用 |
|------|------|------|
| `motion_model_` | `MotionModel` | 底盘运动模型（差速/阿克曼/全向） |
| `critic_manager_` | `CriticManager` | 管理所有 Critic 插件，评估轨迹代价 |
| `noise_generator_` | `NoiseGenerator` | 生成高斯噪声 |
| `trajectory_validator_` | 插件 | 验证最优轨迹是否安全可行 |
| `settings_` | `OptimizerSettings` | 所有超参数 |
| `state_` | `State` | 当前状态 + 速度矩阵 (batch×steps) |
| `control_sequence_` | `ControlSequence` | 最优控制序列 (1×steps) |
| `control_history_` | `Control[4]` | 最近 4 次 SGF 滤波的控制历史 |
| `generated_trajectories_` | `Trajectories` | 所有 batch 条轨迹的 x/y/yaw |
| `path_` | `Path` | 全局规划路径的 Tensor 表示 |
| `costs_` | `ArrayXf` | 每条轨迹的代价 (batch 维) |
| `critics_data_` | `CriticData` | 传给 Critic 的聚合数据包 |

---

## 5. 设计要点

1. **`controller_period == model_dt` 强制约束**：`setOffset()` 中两者不相等直接抛异常，保证控制步长与模型步长一致

2. **双约束体系**：
   - `applyControlSequenceInterIterationConstraints()` — 用 `controller_period` 约束 t=0（当前帧到下一帧之间）
   - `applyControlSequenceConstraints()` — 用 `model_dt` 约束 t≥1（轨迹内部各步之间）

3. **Savitzky-Golay 平滑**：`control_history_[4]` 保存最近 4 帧控制量，SGF 滤波器抑制高频抖动

4. **fallback 重试机制**：全部轨迹碰撞时，reset 后重试（最多 `retry_attempt_limit` 次），超出则抛 `NoValidControl` 异常

