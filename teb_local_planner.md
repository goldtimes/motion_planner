# TEB Local Planner 工程分析

> 分析对象：[rst-tu-dortmund/teb_local_planner](https://github.com/rst-tu-dortmund/teb_local_planner)
> 作者：Christoph Rösmann (TU Dortmund)

---

## 1. 工程概览

```
teb_local_planner/
├── CMakeLists.txt              # 构建配置（依赖 g2o + SuiteSparse）
├── package.xml                 # ROS 包描述
├── teb_local_planner_plugin.xml  # pluginlib 插件注册
├── cfg/
│   └── TebLocalPlannerReconfigure.cfg   # dynamic_reconfigure
├── msg/
│   ├── TrajectoryPointMsg.msg  # 轨迹点消息
│   ├── TrajectoryMsg.msg       # 完整轨迹消息
│   └── FeedbackMsg.msg         # 规划反馈消息
├── include/teb_local_planner/
│   ├── teb_local_planner_ros.h     # ROS 封装层（插件接口）
│   ├── planner_interface.h         # 规划器抽象接口
│   ├── optimal_planner.h           # 核心优化规划器（g2o）
│   ├── homotopy_class_planner.h    # 同伦类并行规划器
│   ├── timed_elastic_band.h        # 轨迹数据结构（TEB）
│   ├── teb_config.h                # 全部配置参数
│   ├── obstacles.h                 # 障碍物模型（点/圆/线/多边形）
│   ├── robot_footprint_model.h     # 机器人足迹模型
│   ├── visualization.h             # 可视化
│   ├── recovery_behaviors.h        # 振荡检测
│   ├── pose_se2.h                  # SE2 位姿
│   ├── misc.h                      # 工具函数
│   ├── distance_calculations.h     # 距离计算
│   ├── h_signature.h               # 同伦类 H-Signature
│   ├── equivalence_relations.h     # 等价关系
│   ├── graph_search.h              # 图搜索（同伦类探索）
│   └── g2o_types/                  # g2o 顶点/边定义
│       ├── vertex_pose.h           # 位姿顶点 (x, y, theta)
│       ├── vertex_timediff.h       # 时间差顶点 (dt)
│       ├── base_teb_edges.h        # 边基类
│       ├── edge_obstacle.h         # 避障代价边
│       ├── edge_dynamic_obstacle.h # 动态障碍物代价边
│       ├── edge_velocity.h         # 速度约束边
│       ├── edge_acceleration.h     # 加速度约束边
│       ├── edge_time_optimal.h     # 时间最优代价边
│       ├── edge_kinematics.h       # 运动学约束边
│       ├── edge_shortest_path.h    # 路径最短边
│       ├── edge_via_point.h        # Via-point 约束边
│       ├── edge_prefer_rotdir.h    # 偏好转向方向边
│       ├── edge_velocity_obstacle_ratio.h  # 速度-障碍物比例边
│       └── penalties.h             # 惩罚函数
├── src/
│   ├── teb_local_planner_ros.cpp   # ROS 封装实现
│   ├── optimal_planner.cpp         # g2o 优化求解实现
│   ├── timed_elastic_band.cpp      # TEB 轨迹操作实现
│   ├── homotopy_class_planner.cpp  # 同伦类规划实现
│   ├── obstacles.cpp               # 障碍物模型实现
│   ├── teb_config.cpp              # 配置加载实现
│   ├── visualization.cpp           # 可视化实现
│   ├── recovery_behaviors.cpp      # 振荡检测实现
│   ├── graph_search.cpp            # 图搜索实现
│   └── test_optim_node.cpp         # 独立测试节点
├── launch/
│   └── teb_local_planner.launch
├── scripts/
├── test/
├── cmake_modules/                  # FindG2O, FindSuiteSparse
├── README.md
└── CHANGELOG.rst
```

---

## 2. 软件架构层次

```
┌─────────────────────────────────────────────────────────────────┐
│                   ROS Navigation Stack                          │
│  move_base / move_base_flex (MBF)                               │
└──────────────────────────┬──────────────────────────────────────┘
                           │ pluginlib 加载
┌──────────────────────────▼──────────────────────────────────────┐
│  TebLocalPlannerROS          (ROS 封装层)                        │
│  - 继承 nav_core::BaseLocalPlanner                              │
│  - 继承 mbf_costmap_core::CostmapController                     │
│  - initialize / setPlan / computeVelocityCommands / isGoalReached│
│  - pruneGlobalPlan / transformGlobalPlan / 障碍物管理 / 可视化   │
│  - configureBackupModes / saturateVelocity                      │
└────────────────────────────┬────────────────────────────────────┘
                             │
            ┌────────────────┼────────────────┐
            ▼                ▼                ▼
┌────────────────────┐ ┌──────────────────┐ ┌──────────────────┐
│  TebOptimalPlanner  │ │HomotopyClassPlanner│ │  TebVisualization │
│  (单轨迹优化)       │ │(多拓扑并行规划)    │ │  (RViz 可视化)    │
│  - g2o 稀疏优化     │ │- H-Signature 同伦 │ │                    │
│  - 构建/求解图      │ │  类辨识           │ │                    │
│  - 冷启动/热启动    │ │- 多线程并行优化   │ │                    │
└────────┬───────────┘ └────────┬─────────┘ └──────────────────────┘
         │                      │
         ▼                      ▼
┌──────────────────────────────────────────────────────────────────┐
│  TimedElasticBand              (轨迹数据结构)                      │
│  - PoseSequence: [s_i] = (x_i, y_i, theta_i)                    │
│  - TimeDiffSequence: [dt_i]  时间间隔                             │
│  - initTEBtoGoal / updateAndPruneTEB / resample / etc.           │
└──────────────────────────────────────────────────────────────────┘
         │
         ▼
┌──────────────────────────────────────────────────────────────────┐
│  g2o 优化框架             (图优化引擎)                            │
│                                                                  │
│  顶点 (Vertex):                                                  │
│  ┌──────────────────────┐                                        │
│  │ VertexPose (x,y,θ)   │  ← 空间位姿 (n 个)                     │
│  │ VertexTimeDiff (dt)  │  ← 时间间隔 (n-1 个)                   │
│  └──────────┬───────────┘                                        │
│             │                                                     │
│  边 (Edge): 连接顶点, 定义代价函数                                │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │ EdgeObstacle        │ 位姿→障碍物距离 (penaltyBelow)        │  │
│  │ EdgeDynamicObstacle │ 动态障碍物预测                        │  │
│  │ EdgeVelocity        │ s_i, s_i+1, dt → v, w 约束           │  │
│  │ EdgeAcceleration    │ 3 poses + 2 dt → a, α 约束           │  │
│  │ EdgeTimeOptimal     │ dt → 最小化 Σdt²                     │  │
│  │ EdgeShortestPath    │ s_i, s_i+1 → 最小化路径长度           │  │
│  │ EdgeKinematics      │ s_i, s_i+1 → 非完整约束               │  │
│  │ EdgeViaPoint        │ s_i → 经过指定点                      │  │
│  │ EdgePreferRotDir    │ 偏好转向方向（振荡恢复）                │  │
│  └────────────────────────────────────────────────────────────┘  │
│                                                                  │
│  求解器: Levenberg-Marquardt / Gauss-Newton                      │
│  线性求解器: CHOLMOD / CSparse (SuiteSparse)                    │
└──────────────────────────────────────────────────────────────────┘
```

---

## 3. 核心类详解

### 3.1 `TebLocalPlannerROS` — ROS 封装层

**文件**: `teb_local_planner_ros.h / .cpp`

作为 move_base 的插件，继承 `nav_core::BaseLocalPlanner` + `mbf_costmap_core::CostmapController`。

**关键流程** (`computeVelocityCommands`):

```
1. 获取机器人位姿 & 速度 (costmap + odom)
2. pruneGlobalPlan()     → 裁切车后方的路径点
3. transformGlobalPlan() → 将全局路径变换到局部 costmap 坐标系
4. 检查是否到达终点 (距离 + 朝向 + 速度)
5. configureBackupModes() → 检查是否需要缩减 horizon 或振荡恢复
6. 清空障碍物容器 → 重新填充:
   ├─ updateObstacleContainerWithCostmap()          (costmap 直接读)
   ├─ updateObstacleContainerWithCostmapConverter()  (插件转换多边形)
   └─ updateObstacleContainerWithCustomObstacles()   (自定义话题)
7. planner_->plan(transformed_plan, &vel)  → g2o 优化
8. 检查发散 (hasDiverged)
9. 检查可行性 (isTrajectoryFeasible → costmap 碰撞检测)
10. getVelocityCommand() → 提取速度指令
11. saturateVelocity()   → 速度饱和限幅
12. 可选: 阿克曼转角转换
13. 可视化
```

### 3.2 `TebOptimalPlanner` — 核心优化规划器

**文件**: `optimal_planner.h / .cpp`

使用 **g2o 图优化框架** 对 TEB 轨迹进行稀疏非线性优化。

**关键方法**:

| 方法 | 作用 |
|------|------|
| `plan(initial_plan, start_vel)` | 主入口，支持热启动 |
| `optimizeTEB()` | 调用 g2o 求解器进行迭代优化 |
| `buildGraph()` | 构建 g2o 超图（添加顶点和边） |
| `addEdgesObstacles()` | 添加障碍物代价边 |
| `addEdgesVelocity()` | 添加速度约束边 |
| `addEdgesAcceleration()` | 添加加速度约束边 |
| `addEdgesTimeOptimal()` | 添加时间最优边 |
| `addEdgesShortestPath()` | 添加路径最短边 |
| `addEdgesKinematics()` | 添加运动学约束边 |
| `addEdgesViaPoints()` | 添加途经点约束边 |
| `getVelocityCommand()` | 从优化结果提取速度指令 |
| `isTrajectoryFeasible()` | 碰撞可行性检查 |
| `clearPlanner()` | 清空规划器状态 |
| `setPreferredTurningDir()` | 设置偏好转向方向（振荡恢复） |

**热启动逻辑**:

```
plan() 调用流程:
1. 如果已有轨迹存在:
   → updateAndPruneTEB() 更新原有轨迹
2. 如果轨迹不存在或距离上次目标太远:
   → initTEBtoGoal() 重新初始化
   → 插入临时状态（如果热启动失败）
3. 重新采样轨迹 (autoResize)
4. buildGraph() → g2o::Solver → optimizeTEB()
5. 计算代价
```

### 3.3 `TimedElasticBand` — 轨迹数据结构

**文件**: `timed_elastic_band.h / .cpp`

定义 TEB 轨迹的核心数据结构和操作方法。

**数据结构**:
```
PoseSequence:     [s_0, s_1, ..., s_n]     → VertexPose (x, y, theta)
TimeDiffSequence: [dt_0, dt_1, ..., dt_n-1] → VertexTimeDiff
```

**关键方法**:

| 方法 | 作用 |
|------|------|
| `initTEBtoGoal(start, goal)` | 在起点和终点间线性插值初始化 |
| `updateAndPruneTEB(plan, min_dist)` | 用新全局计划更新已有轨迹 |
| `autoResize(dt_ref, dt_hysteresis, min_samples, max_samples)` | 自动调整轨迹分辨率 |
| `insertPose()` / `deletePose()` | 增删位姿 |
| `computeCost()` | 计算当前轨迹总代价 |

### 3.4 `HomotopyClassPlanner` — 同伦类并行规划

**文件**: `homotopy_class_planner.h / .cpp`

当启用 `hcp.enable_homotopy_class_planning` 时替代 `TebOptimalPlanner`。

**核心思想**：
- 采样多条不同拓扑（同伦类）的候选路径
- 用 **H-Signature** 区分不同的同伦类（绕障碍物左侧/右侧）
- 每个同伦类保留一个候选，并行优化
- 选择代价最小的轨迹作为最终结果

**关键方法**:

| 方法 | 作用 |
|------|------|
| `plan(initial_plan, ...)` | 探索同伦类、并行优化、选最优 |
| `exploreHomotopyClasses()` | 生成/采样候选路径 |
| `filterEquivalenceClasses()` | H-Signature 同伦类过滤 |
| `findBestPlanner()` | 选择代价最小的规划结果 |
| `getVelocityCommand()` | 从最优规划器提取速度 |

**同伦类探索流程**:
```
1. 生成初始候选: 从全局路径偏移采样
2. 对每个候选计算 H-Signature
3. 同伦类过滤: 每个同伦类只保留一个候选
4. 对保留的候选进行 TEB 优化 (并行)
   - 新的候选 → 冷启动
   - 上一帧已有的候选 → 热启动
5. 比较各轨迹代价, 选最优
```

### 3.5 `Obstacle` 体系 — 障碍物模型

**文件**: `obstacles.h / .cpp`

多态障碍物模型层次:

```
Obstacle (抽象基类)
├── PointObstacle        — 点障碍物
├── CircularObstacle     — 圆形障碍物（有半径）
├── LineObstacle         — 线段障碍物
├── PolygonObstacle      — 多边形障碍物
└── DynamicObstacle      — 动态障碍物（常速模型预测）
```

每个障碍物支持:
- `getMinimumDistance(position)` — 点到障碍物最短距离
- `checkCollision(position, min_dist)` — 碰撞检测
- `checkLineIntersection(line_start, line_end)` — 线段相交检测
- `getCentroidVelocity()` — 动态障碍物速度

### 3.6 `RobotFootprintModel` — 机器人足迹模型

**文件**: `robot_footprint_model.h`

多态足迹模型层次:

```
RobotFootprintModel (抽象基类)
├── PointRobotFootprint         — 点模型
├── CircularRobotFootprint      — 圆形模型
├── LineRobotFootprint          — 线段模型（前后轮）
├── TwoCirclesRobotFootprint    — 双圆模型（前/后）
└── PolygonRobotFootprint       — 多边形模型（与 costmap 一致）
```

---

## 4. g2o 优化图结构

### 顶点 (Vertices)

```
VertexPose(i)     — 第 i 个位姿, 参数 [x, y, theta] (3 DOF)
VertexTimeDiff(i) — 第 i 个时间间隔, 参数 [dt] (1 DOF)
```

### 边 (Edges) — 代价函数

| 边 | 连接的顶点 | 残差维数 | 作用 |
|---|-----------|---------|------|
| `EdgeObstacle` | 1 个 VertexPose | 1 | 避障: penaltyBelow(distance - min_dist) |
| `EdgeDynamicObstacle` | 1 个 VertexPose | 1 | 动态障碍物预测避障 |
| `EdgeVelocity` | 2 个 VertexPose + 1 VertexTimeDiff | 2 | 速度约束: 区间惩罚 [v, w] |
| `EdgeAcceleration` | 3 个 VertexPose + 2 VertexTimeDiff | 2 | 加速度约束: 区间惩罚 [a, α] |
| `EdgeTimeOptimal` | 1 个 VertexTimeDiff | 1 | 最小化总时间: dt² |
| `EdgeShortestPath` | 2 个 VertexPose | 1 | 最小化路径长度 |
| `EdgeKinematics` | 2 个 VertexPose | 2 | 非完整约束（航向对齐运动方向）|
| `EdgeViaPoint` | 1 个 VertexPose | 1 | 途经点约束 |
| `EdgePreferRotDir` | 1 个 VertexPose | 1 | 偏好转向方向（振荡恢复）|
| `EdgeVelocityObstacleRatio` | 2 个 VertexPose + 1 VertexTimeDiff | 1 | 靠近障碍物时减速 |

### 惩罚函数 (`penalties.h`)

所有约束使用 **软约束**（惩罚函数）而非硬约束:

```
penaltyBoundFromBelow(x, limit)  — 当 x < limit 时触发惩罚
penaltyBoundFromAbove(x, limit)  — 当 x > limit 时触发惩罚
penaltyBoundToInterval(x, lower, upper) — 当 x ∉ [lower, upper] 时触发惩罚
penaltyEquality(x, target)       — 等式约束惩罚
```

### 求解器配置

```cpp
typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>> TEBBlockSolver;
typedef g2o::LinearSolverCSparse<TEBBlockSolver::PoseMatrixType> TEBLinearSolver;

// 优化算法: Levenberg-Marquardt (默认) 或 Gauss-Newton
Solver = new g2o::OptimizationAlgorithmLevenberg(solver);
```

---

## 5. 配置参数体系 (`TebConfig`)

| 参数分组 | 主要参数 | 说明 |
|---------|---------|------|
| **Trajectory** | `dt_ref`, `teb_autosize`, `min_samples`, `max_samples` | 轨迹分辨率、自动调整 |
|  | `global_plan_overwrite_orientation` | 是否覆盖全局路径朝向 |
|  | `max_global_plan_lookahead_dist` | 最大前视距离 |
|  | `global_plan_viapoint_sep` | 从全局路径提取 via-point 的最小间隔 |
|  | `control_look_ahead_poses` | 提取速度指令的位姿索引 |
| **Robot** | `max_vel_x/y/trans/theta`, `acc_lim_x/y/theta` | 速度/加速度限制 |
|  | `min_turning_radius`, `wheelbase` | 阿克曼车型参数 |
|  | `cmd_angle_instead_rotvel` | 是否用转向角代替角速度 |
| **GoalTolerance** | `xy_goal_tolerance`, `yaw_goal_tolerance` | 终点容忍度 |
|  | `free_goal_vel` | 终点是否允许非零速度 |
| **Obstacles** | `min_obstacle_dist` | 最小避障距离 |
|  | `inflation_dist` | 障碍物膨胀距离（软约束） |
|  | `costmap_converter_plugin` | costmap 转多边形插件 |
|  | `obstacle_poses_affected` | 每个障碍物影响的位姿数 |
|  | `legacy_obstacle_association` | 新旧障碍物关联策略 |
| **Optimization** | `optimize_*` (各代价的激活/权重) | 每个代价项可单独开关和设权重 |
|  | `weight_optimaltime` | 时间最优权重 |
|  | `weight_shortest_path` | 路径最短权重 |
|  | `weight_obstacle` | 避障权重 |
|  | `weight_kinematics` | 运动学约束权重 |
|  | `weight_acceleration` | 加速度权重 |
|  | `weight_viapoint` | 途经点权重 |
| **HCP** | `enable_homotopy_class_planning` | 启用同伦类规划 |
|  | `simple_exploration` / `exploration_*` | 同伦类探索参数 |
|  | `max_number_classes` | 最大同伦类数量 |
|  | `roadmap_graph_*` | 路标图参数 |
| **Recovery** | `shrink_horizon_backup` | 失败时缩减 horizon |
|  | `oscillation_recovery` | 振荡恢复策略 |

---

## 6. 与用 Ceres 实现的版本对比

| 方面 | 官方 TEB (g2o) | Ceres 实现 |
|------|---------------|-----------|
| **优化引擎** | g2o（图优化框架） | Ceres（非线性最小二乘） |
| **顶点/参数块** | VertexPose(x,y,θ) + VertexTimeDiff(dt) 分开 | 单个参数块 [x,y,θ,dt] |
| **边/代价函数** | 每个约束一个 Edge 类，多态 | 每个约束一个 CostFunctor |
| **障碍物模型** | Point/Circular/Line/Polygon (多态) | costmap 栅格代价 |
| **机器人足迹** | 5 种足迹模型 | 无（可用 costmap footprint） |
| **微分方式** | 解析 Jacobian (USE_ANALYTIC_JACOBI) | Ceres AutoDiff (自动微分) |
| **求解器** | CHOLMOD / CSparse (SuiteSparse) | SPARSE_NORMAL_CHOLESKY |
| **同伦类规划** | ✅ HomotopyClassPlanner | ❌ |
| **动态障碍物** | ✅ 常速模型预测 | ❌ |
| **backup 模式** | ✅ horizon 缩减 + 振荡恢复 | ❌ |
| **可行性检查** | ✅ costmap_model 精确碰撞检测 | ❌ |
| **热启动** | ✅ updateAndPruneTEB | 无 |
| **阿克曼车型** | ✅ cmd_angle_instead_rotvel | ❌ |
| **实时性** | 成熟稳定，参数丰富 | 轻量，依赖 Ceres 效率 |

---

## 7. 关键技术细节

### 7.1 TEB 轨迹初始化 (`initTEBtoGoal`)

```
1. 确定起点 (s_start) 和终点 (s_goal)
2. 计算欧氏距离 L = ||s_goal - s_start||
3. 根据 dt_ref 确定初始位姿数量 N = ceil(L / (v_max * dt_ref))
4. 在起点和终点之间均匀插值生成 N 个位姿
5. 每个 dt = L / (v_max * N)  或  设置为 dt_ref
6. 将数据封装为 VertexPose 和 VertexTimeDiff
```

### 7.2 障碍物关联 (`obstacle_poses_affected`)

每个障碍物不会连接所有位姿（计算量太大），而是：
- 找到距离障碍物最近的 TEB 位姿
- 连接该位姿及其前后 `obstacle_poses_affected` 个位姿到障碍物边
- 新策略：对每个位姿，只连接附近 `obstacle_association_force_inclusion_factor * min_obstacle_dist` 内的障碍物

### 7.3 自动调整轨迹分辨率 (`autoResize`)

```
if 平均 dt > dt_ref * (1 + dt_hysteresis):
   在需要的位置插入新位姿（降低 dt）
elif 平均 dt < dt_ref * (1 - dt_hysteresis):
   删除冗余位姿（增大 dt）

保证: min_samples ≤ N ≤ max_samples
```

### 7.4 速度指令提取 (`getVelocityCommand`)

从优化后的轨迹提取速度指令：
```
1. 取第 control_look_ahead_poses 个位姿作为目标
2. 计算从当前位置到目标位置的位移
3. v = displacement / dt  (使用 exact_arc_length 或欧氏距离)
4. ω = angle_diff / dt
5. 对速度进行饱和限幅
```

### 7.5 可行性检查 (`isTrajectoryFeasible`)

```
对轨迹上的每个位姿（直到 feasibility_check_no_poses 或
feasibility_check_lookahead_distance）:
  1. 将机器人 footprint 投影到 costmap
  2. 检查 footprint 内是否有 LETHAL_OBSTACLE 代价
  3. 如果有 → 轨迹不可行
```

### 7.6 同伦类探索 (`exploreHomotopyClasses`)

```
1. 生成探索候选:
   - 沿全局路径法线方向采样偏移
   - 使用路标图 (roadmap_graph) 搜索多条路径
2. 对每个候选计算 H-Signature:
   - HSignature = Σ(angle_to_obstacle_i) 绕障碍物的总角度
3. 用 H-Signature 过滤: 同一个同伦类只保留一个候选
4. 对保留候选分别进行 TEB 优化
5. 选择总代价最小的轨迹
```

---

## 8. 优点与局限性

### 优点
- **成熟稳定**：广泛使用，参数可调性极强
- **同伦类规划**：能跳出局部最优，选择最佳拓扑
- **柔性约束**：全部使用软约束，优化稳定性好
- **多种机器人支持**：差速/全向/阿克曼
- **动态障碍物**：支持常速预测
- **热启动**：帧间连续性良好

### 局限性
- **高度参数依赖**：约 100 个参数，调参复杂
- **g2o 依赖**：g2o 维护不如 Ceres 活跃
- **计算量**：同伦类规划在多障碍物场景下可能较慢
- **软约束**：速度/加速度约束可能被轻微违反
- **局部极小值**：单 TEB 仍可能陷入局部最优（但同伦类可缓解）

---

## 9. 对你的 Ceres 实现的改进建议

基于对官方 TEB 的分析，建议优先补充以下能力：

1. **backup 模式**：连续规划失败时自动缩短 horizon
2. **可行性检查**：优化后做 costmap 精确碰撞检测
3. **热启动**：帧间复用上一步优化结果作为初始值
4. **障碍物关联**：只连接附近障碍物，降低计算量
5. **自动分辨率调整**：根据 dt_ref 自动增删位姿
6. **多种足迹模型**：支持更多机器人外形
7. **速度指令提取**：使用 `control_look_ahead_poses` 而非固定取前两个位姿
