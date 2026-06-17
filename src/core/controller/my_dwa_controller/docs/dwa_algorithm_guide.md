# DWA (Dynamic Window Approach) 算法原理与调参指南

> 对应代码实现：`my_dwa_controller`
> 代码位置：`src/core/controller/my_dwa_controller/`

---

## 一、算法总览

DWA 是一种**速度空间搜索**的局部路径规划方法。核心思想：

> **每帧只在"当前速度附近、当前加速度能到达"的范围内采样速度，模拟出轨迹，选最好的一条执行。**

它不是全局搜遍所有速度，而是只搜一个 **"动态窗口"**——这就是算法名字的由来。

### 输入 / 输出

```
输入:
  - 全局路径 (global_plan_)
  - 当前位姿 (x, y, theta)   ← costmap
  - 当前速度 (vx, vy, vth)   ← odometry
  - 代价地图 (costmap)        ← 激光/深度传感器

输出:
  - 速度指令 (cmd_vel.linear.x, cmd_vel.angular.z)
  - 局部路径 (local_plan，用于可视化)
```

---

## 二、算法六步流程

### Step 1 — 获取当前状态

读取机器人当前位姿 `(x, y, θ)` 和速度 `(v, ω)`。

**你在代码中的位置**：`computeVelocityCommands()` 前半部分。

```cpp
// 获取位姿
costmap_ros_->getRobotPose(current_pose_);
const double px = current_pose_.pose.position.x;
const double py = current_pose_.pose.position.y;
const double pth = tf2::getYaw(current_pose_.pose.orientation);

// 获取速度
odom_helper_->getRobotVel(robot_vel_msg);
const double vx = robot_vel_msg.pose.position.x;   // 线速度
const double vth = tf2::getYaw(robot_vel_msg.pose.orientation); // 角速度
```

### Step 2 — 计算动态窗口 (Dynamic Window)

动态窗口是**三个约束的交集**：

| 约束 | 含义 | 公式 |
|------|------|------|
| **(a) 运动学约束** | 机器人硬件能跑多快 | $v \in [v_{\min}, v_{\max}]$, $\omega \in [\omega_{\min}, \omega_{\max}]$ |
| **(b) 动力学约束** | 一个控制周期内能加速/减速到多少 | $v \in [v_t - a_v \Delta t,\; v_t + a_v \Delta t]$ |
| **(c) 安全约束** | 碰到障碍前能刹停的速度（可选） | $v \leq \sqrt{2 \cdot d \cdot a_{\text{brake}}}$ |

**代码位置**：`calcDynamicWindow()`

```
动态窗口 = (a) ∩ (b) ∩ (c)
```

注意：我们的简化实现只取了 **(a) ∩ (b)**，没有实现 (c) 刹车距离约束。这是未来可以改进的地方。

> **💡 为什么叫"动态"？** —— 因为窗口随着当前速度变化而变化。机器人静止时窗口小，跑起来后窗口跟着移动。

### Step 3 — 速度采样 (Velocity Sampling)

把动态窗口离散化为网格，均匀采样：

```cpp
// vx_samples = 6, vtheta_samples = 20 → 6×20 = 120 个候选速度
for (int i = 0; i < n_v; ++i)
  for (int j = 0; j < n_w; ++j)
    samples.emplace_back(v, w);
```

**代码位置**：`sampleVelocities()`

### Step 4 — 轨迹预测 (Trajectory Prediction)

对每个候选速度 `(v, ω)`，用**单轮车模型 (unicycle model)** 前向模拟 $\text{sim\_time}$ 秒：

$$
\begin{aligned}
x_{t+1} &= x_t + v \cdot \cos(\theta_t) \cdot \Delta t \\
y_{t+1} &= y_t + v \cdot \sin(\theta_t) \cdot \Delta t \\
\theta_{t+1} &= \theta_t + \omega \cdot \Delta t
\end{aligned}
$$

- $\Delta t = 0.05$s (默认)，一共模拟 `sim_time / dt` 步
- 两个相邻点之间还会以 `sim_granularity = 0.025`m 的分辨率进行**线性插值**，确保小障碍不被漏掉

**代码位置**：`predictTrajectory()`

### Step 5 — 代价评估 (Cost Evaluation)

对每条轨迹计算加权总代价：

$$
\text{total} = w_{\text{occ}} \cdot \text{occ\_cost} + w_{\text{path}} \cdot \text{path\_cost} + w_{\text{goal}} \cdot \text{goal\_cost} + w_{\text{speed}} \cdot \text{speed\_cost}
$$

#### 五项代价详解

| 代价项 | 函数 | 含义 | 输入 |
|--------|------|------|------|
| **障碍物代价** `occ_cost` | `calcObstacleCost()` | 轨迹上最靠近障碍物那一点的归一化代价 | costmap |
| **路径对齐代价** `path_cost` | `calcPathCost()` | 轨迹上各点到全局路径的平均距离 | global_plan |
| **目标距离代价** `goal_cost` | `calcGoalCost()` | 轨迹终点到全局终点的欧氏距离 | global_plan.back() |
| **速度鼓励代价** `speed_cost` | `calcSpeedCost()` | $v_{\max} - \|v\|$，越慢惩罚越大 | 候选速度 v |

障碍物代价的特殊设计（`calcObstacleCost`）：

```
对每个插值点:
  ├─ 中心点查 costmap
  ├─ 左偏 robot_radius 查 costmap  → 机器人足迹检测
  └─ 右偏 robot_radius 查 costmap
       │
       ├─ 任意点 cost ≥ INSCRIBED_INFLATED → 轨迹无效 (return -1)
       │
       └─ 否则: max_norm = max(max_norm, cost / LETHAL)
       
最终惩罚: penalty = max_norm² / (1 - max_norm + ε)
          ↘ 离障碍越近 → 惩罚爆炸式增长
```

### Step 6 — 选出最优轨迹

```
best_traj = argmin(total_cost)
     ↓
cmd_vel.linear.x = best_traj.v
cmd_vel.angular.z = best_traj.w
```

如果所有轨迹都无效（`best_traj.cost = inf`）→ 返回 `false`，命令零速度，让 `move_base` 重试。

**代码位置**：`computeVelocityCommands()` 后半部分

---

## 三、完整流程图

```mermaid
flowchart TD
    A[开始控制周期] --> B[获取位姿 + 速度]
    B --> C{已到达目标?}
    C -->|是| D[停止, 返回 true]
    C -->|否| E[计算动态窗口]
    E --> F[采样候选速度 (v, ω)]
    F --> G[对每个候选速度]
    G --> H[预测轨迹 (unicycle模型)]
    H --> I[评估代价]
    I --> J{所有速度已评估?}
    J -->|否| G
    J -->|是| K[选择最优轨迹]
    K --> L{最优轨迹有效?}
    L -->|是| M[输出速度指令]
    L -->|否| N[输出零速度, 返回 false]
    M --> O[发布可视化]
    O --> P[结束]
    N --> P
```

---

## 四、参数调优指南

### 4.1 速度限制

| 参数 | 默认值 | 说明 | 调参建议 |
|------|--------|------|---------|
| `max_vel_x` | 0.55 m/s | 最大前进速度 | 室内小机器人 0.3~0.8；户外可更大 |
| `min_vel_x` | -0.1 m/s | 最小(后退)速度 | 负值允许倒车；碰撞时设为 0 不让后退 |
| `max_vel_theta` | 1.0 rad/s | 最大旋转速度 | 需要灵巧转向时增大到 1.5~2.0 |

### 4.2 加速度限制

| 参数 | 默认值 | 说明 | 调参建议 |
|------|--------|------|---------|
| `acc_lim_x` | 2.5 m/s² | 直线加速度 | 越大加减速越快，但容易打滑 |
| `acc_lim_theta` | 3.2 rad/s² | 角加速度 | 越大转向越灵敏 |

> 加速度决定**动态窗口的宽度**。加速度越小 → 窗口越窄 → 每帧只能微调速度 → 轨迹更平滑但响应慢。

### 4.3 预测参数 ⭐ 最关键

| 参数 | 默认值 | 说明 | 调参建议 |
|------|--------|------|---------|
| `sim_time` | 3.0 s | 前向模拟时长 | **避障不足 → 增大**(4~5s)；**卡顿/反应慢 → 减小**(1.5~2s) |
| `dt` | 0.05 s | 模拟时间步长 | 越小轨迹越精细；通常 0.05~0.1 |
| `sim_granularity` | 0.025 m | 插值检测分辨率 | 越小检测越密但越慢；0.01~0.05 |

**`sim_time` 是避障效果的第一关键参数**：
- **太短**（< 1.5s）：看不见远处的障碍，近前才急刹 → 撞上
- **太长**（> 5s）：计算量大，且会看到过于遥远的路径导致奇怪行为
- **建议**：在空旷场景 2~3s，复杂狭窄场景 3~4s

### 4.4 采样分辨率

| 参数 | 默认值 | 说明 | 调参建议 |
|------|--------|------|---------|
| `vx_samples` | 6 | 线速度采样数 | 越大越细但越慢；一般 6~10 |
| `vtheta_samples` | 20 | 角速度采样数 | 越大转向越细腻；一般 10~20 |

> 总采样数 = `vx_samples × vtheta_samples`。120 个已经有一定计算量了，如果机器人反应迟钝可以减到 80 个。

### 4.5 代价权重 ⭐ 第二关键

| 权重 | 默认值 | 说明 | 调参建议 |
|------|--------|------|---------|
| `obstacle_cost_weight` | 10.0 | 避障权重 | **撞障碍 → 增大**(15~20)；**过于保守不动 → 减小**(3~5) |
| `path_cost_weight` | 1.0 | 跟踪路径权重 | 路径偏离大 → 增大(2~5) |
| `goal_cost_weight` | 0.5 | 趋向目标权重 | 目标吸引不足 → 增大(1~2) |
| `speed_cost_weight` | 0.1 | 速度鼓励 | 机器人太慢 → 增大(0.3~0.5) |

**权重调优逻辑**（最常见的调优场景）：

```
撞障碍物
    ↓
增大 obstacle_cost_weight
如果还撞 → 增大 sim_time (看得更远)
如果还撞 → 减小 max_vel_x (慢一点便于避障)

过于保守，不敢动
    ↓
减小 obstacle_cost_weight
增大 speed_cost_weight
增大 max_vel_x

路径跟踪不好，走大弯
    ↓
增大 path_cost_weight
减小 goal_cost_weight (防止切角)

在目标附近来回震荡
    ↓
减小 goal_cost_weight
检查 xy_goal_tolerance 是否合适
```

### 4.6 避障参数

| 参数 | 默认值 | 说明 | 调参建议 |
|------|--------|------|---------|
| `min_obstacle_dist` | 0.5 m | 距障碍最小安全距离 | 根据机器人大小调整 |
| `max_obstacle_range` | 3.0 m | 障碍物检测最大范围 | 与 sim_time × max_vel_x 匹配 |
| `robot_radius` | 0.15 m | 机器人半径（用于足迹检测） | TurtleBot3 Burger ~0.1, Waffle ~0.15 |

---

## 五、常见问题诊断

### 问题 1：不动 / 总是无合法轨迹

**终端输出**：`MyDWA: No valid trajectory found out of 120 samples. Stopping.`

**可能原因**：
1. 起点被障碍物包围（放在空旷处测试）
2. `INSCRIBED_INFLATED` 阈值太严格 → 减小 costmap 的 `inflation_radius`
3. 动态窗口太小 → 检查 `acc_lim_x` 和 `acc_lim_theta` 是否合理
4. 机器人半径偏置导致检测到自身 footprint → 减小 `robot_radius`

### 问题 2：撞障碍物

**可能原因**：
1. `sim_time` 太短 → 增大
2. `obstacle_cost_weight` 太小 → 增大
3. `max_vel_x` 太大 → 减小，让机器人慢一点
4. 速度太快刹不住 → 考虑添加刹车距离约束 (c)

### 问题 3：路径跟不住

**可能原因**：
1. `path_cost_weight` 太小 → 增大
2. `goal_cost_weight` 太大 → 减小（目标太强的吸引会导致切角抄近路）
3. 全局路径本身不平滑 → 检查全局规划器

### 问题 4：抖动 / 震荡

**可能原因**：
1. `vx_samples` 或 `vtheta_samples` 太少 → 增大采样数
2. `obstacle_cost_weight` 太大 → 适当减小
3. `sim_time` 和 $v_{\max}$ 不匹配 → 让模拟距离覆盖合理范围

---

## 六、调试小技巧

### 可视化所有探索轨迹

在 RViz 中添加 `MarkerArray` 话题：
```
/move_base/MyDWAController/explored_trajectories
```
- **绿色**轨迹 = 有效（可执行）
- **红色**轨迹 = 无效（会撞障碍物）
- 如果**全红没有绿** → 障碍检测太严格
- 如果**很多绿但选了撞墙的** → 权重不对

### 日志分析

终端会每周期打印：
```
MyDWA: 85/120 valid, best cost=0.32 (v=0.22, w=0.05)
```
- `85/120` = 有效轨迹数 / 总采样数
- `best cost` = 最优轨迹总代价（越低越好）
- `v / w` = 输出的线速度和角速度
