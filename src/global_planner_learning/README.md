# 全局路径规划算法

## 概述

本项目基于 ROS + `nav_msgs::OccupancyGrid` 实现了 **A\*** 和 **Dijkstra** 两种全局路径规划算法，均支持 4/8 连通邻域搜索以及未知区域穿越控制。两种算法通过同一基类 `PathPlannerBase` 多态调用，可在运行时通过参数 `planner_name` 自由切换。

本文档除介绍已实现的算法外，还对 **RRT（Rapidly-exploring Random Tree）** 及其改进算法进行概念性说明，为后续扩展实现提供参考。

## 目录结构

```
global_planner_learning/
├── include/global_planner_learning/
│   ├── planner_base.h          # 规划器抽象基类 & 公共数据结构
│   ├── astar_planner.h          # A* 规划器头文件
│   ├── dijkstra_planner.h       # Dijkstra 规划器头文件
│   └── rrt_planner.h            # RRT 规划器头文件
├── src/
│   ├── astar_planner.cpp        # A* 核心算法实现
│   ├── dijkstra_planner.cpp     # Dijkstra 核心算法实现
│   ├── rrt_planner.cpp          # RRT 核心算法实现
│   └── global_planner_node.cpp  # 节点主程序（支持规划器切换）
├── config/
│   └── planner_params.yaml      # 规划器参数配置
└── README.md                    # 本文档
```

## 数据结构

### `Node`（栅格搜索节点）

定义于 `planner_base.h`，是所有栅格搜索算法的基本单元：

| 成员 | 类型 | 说明 |
|------|------|------|
| `x, y` | `int` | 栅格坐标（列，行） |
| `g` | `double` | 从起点到当前节点的实际代价值 |
| `h` | `double` | 当前节点到目标节点的启发式估计值（Dijkstra 中恒为 0） |
| `f` | `double` | 总代价值，`f = g + h`（Dijkstra 中 `f = g`） |
| `parent_x, parent_y` | `int` | 父节点栅格坐标（用于路径回溯） |

比较运算符 `operator>` 按 `f` 值排序，用于最小堆。

### `PlannerStatistics`（规划统计）

每次 `makePlan` 调用后记录：

- `planner_name` — 规划器名称
- `success` — 是否成功
- `planning_time_ms` — 规划耗时（毫秒）
- `visited_nodes_count` — 搜索过程中访问的节点数
- `path_points_count` — 生成的路径点数
- `path_length_m` — 路径总长度（米）

## A* 算法流程

```
┌─────────────────────────────────────────────────┐
│                  输入：起点 & 终点                 │
│            (世界坐标 → 栅格坐标转换)                │
└────────────────────┬────────────────────────────┘
                     ▼
┌─────────────────────────────────────────────────┐
│           Step 1: 检查起点/终点合法性              │
│            - 是否在地图范围内                      │
│            - 是否可通行（不在障碍物上）              │
└────────────────────┬────────────────────────────┘
                     ▼
┌─────────────────────────────────────────────────┐
│          Step 2: 初始化搜索数据结构                 │
│   ┌─────────────────────────────────────────┐    │
│   │ • closed_set[] ← false (全部未关闭)      │    │
│   │ • g_value[] ← ∞ (代价值初始化为无穷大)     │    │
│   │ • parent_[] ← {-1, -1} (无父节点)         │    │
│   │ • open_set ← 空优先队列(最小堆)            │    │
│   │ • 创建起点节点:                            │    │
│   │   g = 0, h = heuristic(), f = g + h      │    │
│   │ • open_set.push(起点)                     │    │
│   │ • g_value[起点] = 0                       │    │
│   └─────────────────────────────────────────┘    │
└────────────────────┬────────────────────────────┘
                     ▼
┌─────────────────────────────────────────────────┐
│          Step 3: A* 主循环                        │
│                                                 │
│  while (open_set 不为空) {                       │
│                                                 │
│    ① current = open_set.top()   ← 取出 f 最小节点 │
│       open_set.pop()                             │
│                                                 │
│    ② if current 已在 closed_set 中 → continue    │
│       (已被更优路径处理过)                        │
│                                                 │
│    ③ closed_set[current] = true                 │
│       parent_[current] = current.parent          │
│                                                 │
│    ④ if current == goal                         │
│         → 重建路径，返回成功                      │
│                                                 │
│    ⑤ 遍历所有邻居 (4/8 方向)                     │
│       ┌──────────────────────────────────┐       │
│       │ • 跳过不可通行或 closed 的邻居     │       │
│       │ • 计算 step_cost                 │       │
│       │   (直线=1.0, 对角≈1.414)         │       │
│       │ • new_g = current.g + step_cost   │       │
│       │ • if new_g ≥ g_value[邻居] → 跳过 │       │
│       │   (已有更优路径)                  │       │
│       │ • 更新 g_value, 创建邻居节点       │       │
│       │ • open_set.push(邻居)             │       │
│       └──────────────────────────────────┘       │
│  }                                               │
│                                                 │
│  若 open_set 为空仍未到达目标 → 返回失败           │
└────────────────────┬────────────────────────────┘
                     ▼
┌─────────────────────────────────────────────────┐
│          Step 4: 路径回溯                         │
│                                                 │
│  从目标节点开始，沿着 parent_ 数组向前追溯          │
│  直到遇到父节点为 {-1, -1}（起点）                 │
│                                                 │
│  将回溯得到的节点序列反转 → 得到从起点到终点的路径    │
└────────────────────┬────────────────────────────┘
                     ▼
┌─────────────────────────────────────────────────┐
│          Step 5: 统计输出 & 返回                   │
│   - 记录规划耗时、访问节点数、路径长度等             │
│   - 通过 ROS 日志输出 (INFO / WARN)               │
│   - 返回 true (成功) 或 false (失败)               │
└─────────────────────────────────────────────────┘
```

## 核心函数详解

### 1. `worldToGrid` / `gridToWorld` — 坐标转换

```
栅格坐标 = (世界坐标 - 地图原点) / 分辨率
世界坐标 = 地图原点 + (栅格坐标 + 0.5) × 分辨率
```

- `worldToGrid`：将世界坐标 `(wx, wy)` 转为栅格坐标 `(gx, gy)`，同时检查是否在地图范围内
- `gridToWorld`：将栅格坐标 `(gx, gy)` 转为世界坐标（取栅格中心点）

### 2. `isWalkable` — 可通行性检查

判断栅格 `(gx, gy)` 是否允许通行：

```
if 超出地图边界 → false
if 栅格值 == UNKNOWN_VALUE (-1) → 返回 allow_unknown_ 配置
if 栅格值 >= OCCUPIED_THRESH (50) → false (障碍物)
else → true
```

### 3. `getHeuristic` — 启发函数

根据连通性选择不同的启发式距离：

| 连通性 | 启发函数 | 说明 |
|--------|----------|------|
| 4 方向 | 曼哈顿距离 `│dx│ + │dy│` | 只允许上下左右移动 |
| 8 方向 | 对角线距离 `max(│dx│, │dy│)` | 允许对角线移动，保证启发式可采纳 |

> 两种启发函数均满足 **可采纳性**（admissible，即估计值 ≤ 实际最小代价），确保 A\* 能找到最优路径。

### 4. `makePlan` — A* 主算法入口

完整的执行步骤：

1. **坐标转换** — 将起点和终点从世界坐标转为栅格坐标
2. **合法性检查** — 确保起点和终点在地图内且可通行
3. **初始化** — 创建 `closed_set`、`g_value`、`parent_` 数组和 `open_set` 优先队列
4. **主循环** — 不断从 `open_set` 中弹出 `f` 值最小的节点，检查是否为目标，否则扩展其邻居
5. **路径回溯** — 从目标节点沿 `parent_` 链回溯到起点，反转得到最终路径
6. **统计输出** — 记录规划耗时、访问节点数、路径长度等信息

### 5. `reconstructPath` — 路径回溯

沿着 `parent_` 数组从目标节点回溯到起点：

- 当前节点 `(cx, cy)` 从目标点开始
- 通过 `parent_[cy * width + cx]` 获取父节点坐标
- 持续回溯直到父节点为 `{-1, -1}`（起点的标记）
- 包含最大迭代保护（`width * height`），防止死循环
- 最后将路径反转，使其从起点到终点排列

### 6. `computePathLength` — 路径长度计算

遍历路径中的连续点对，累加欧氏距离：

```
length = Σ sqrt(Δx² + Δy²)
```

## 关键参数与配置

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `use_8_connectivity_` | `bool` | `true` | `true`=8方向搜索，`false`=4方向搜索 |
| `allow_unknown_` | `bool` | `false` | 是否允许穿越未知区域（地图中值为-1的栅格） |
| `OCCUPIED_THRESH` | `int8_t` | `50` | 障碍物判定阈值（栅格值 > 50 视为障碍物） |
| `UNKNOWN_VALUE` | `int8_t` | `-1` | 未知区域的地图栅格值 |

## 邻域定义

### 4 连通（上下左右）

```
      ↑ (0, -1)
← (-1, 0)  ·  → (1, 0)
      ↓ (0, 1)
```

移动代价：直线 = 1.0

### 8 连通（4 方向 + 对角线）

```
↖ (-1,-1)  ↑ (0,-1)  ↗ (1,-1)
← (-1,0)    ·       → (1,0)
↙ (-1,1)   ↓ (0,1)   ↘ (1,1)
```

移动代价：直线 = 1.0，对角 ≈ 1.414 (√2)

## 算法特点

| 特性 | 说明 |
|------|------|
| **完备性** | 如果存在可行路径，A\* 一定能找到 |
| **最优性** | 在启发函数可采纳（admissible）的前提下，保证找到最短路径 |
| **效率** | 使用启发式 `h` 指导搜索方向，比 Dijkstra 算法更高效 |
| **内存** | 维护 `closed_set`、`g_value`、`parent_` 三个等宽高数组，空间复杂度 O(N) |

## 运行流程示例

```
输入: start=(0.5, 0.5), goal=(9.5, 9.5)

Step 1: 世界→栅格  start=(0,0), goal=(9,9)
Step 2: 检查合法 ✓
Step 3: 初始化 open_set = [起点(0,0)]
Step 4: 主循环
  ├─ 弹出 (0,0), f=18.0, 扩展邻居...
  ├─ 弹出 (1,0), f=17.0, 扩展邻居...
  ├─ 弹出 (0,1), f=17.0, 扩展邻居...
  ├─ ...
  ├─ 弹出 (9,9), f=0.0  → 到达目标!
Step 5: 路径回溯 → 生成路径
Step 6: 输出统计
  [AStar] SUCCESS | time: 2.35 ms | visited: 87 nodes | path: 10 pts | length: 12.73 m
```

## Dijkstra 算法

### 与 A* 的关系

Dijkstra 算法是 A* 算法在没有启发式信息时的特例。两者的核心区别在于：

| 对比维度 | A* | Dijkstra |
|---------|-----|----------|
| 启发函数 | `h(x) ≥ 0`，引导搜索方向 | `h ≡ 0`，无方向引导 |
| 优先队列排序依据 | `f = g + h` | `f = g`（即实际距离） |
| 搜索范围 | 朝向目标方向优先扩展 | 以起点为圆心同心圆状向外扩张 |
| 搜索效率 | 较高（启发式剪枝） | 较低（全向扩展） |
| 最优性 | 保证（h 可采纳时） | 保证 |
| 实现复杂度 | 需设计启发函数 | 无需启发函数 |

### 算法流程

```
┌─────────────────────────────────────────────────┐
│                  输入：起点 & 终点                 │
│            (世界坐标 → 栅格坐标转换)                │
└────────────────────┬────────────────────────────┘
                     ▼
┌─────────────────────────────────────────────────┐
│           Step 1: 检查起点/终点合法性              │
│            - 是否在地图范围内                      │
│            - 是否可通行（不在障碍物上）              │
└────────────────────┬────────────────────────────┘
                     ▼
┌─────────────────────────────────────────────────┐
│          Step 2: 初始化搜索数据结构                 │
│   ┌─────────────────────────────────────────┐    │
│   │ • closed_set[] ← false (全部未关闭)      │    │
│   │ • dist[] ← ∞ (距离初始化为无穷大)          │    │
│   │ • parent_[] ← {-1, -1} (无父节点)         │    │
│   │ • open_set ← 空优先队列(最小堆)            │    │
│   │ • 创建起点节点:                            │    │
│   │   g = 0, h = 0, f = g                    │    │
│   │ • open_set.push(起点)                     │    │
│   │ • dist[起点] = 0                          │    │
│   └─────────────────────────────────────────┘    │
└────────────────────┬────────────────────────────┘
                     ▼
┌─────────────────────────────────────────────────┐
│      Step 3: Dijkstra 主循环                     │
│                                                 │
│  while (open_set 不为空) {                       │
│                                                 │
│    ① current = open_set.top()   ← 取出 g 最小节点│
│       open_set.pop()                             │
│                                                 │
│    ② if current 已在 closed_set 中 → continue    │
│       (已被更优路径处理过)                        │
│                                                 │
│    ③ closed_set[current] = true                 │
│       parent_[current] = current.parent          │
│                                                 │
│    ④ if current == goal                         │
│         → 重建路径，返回成功                      │
│                                                 │
│    ⑤ 遍历所有邻居 (4/8 方向)                     │
│       ┌──────────────────────────────────┐       │
│       │ • 跳过不可通行或 closed 的邻居     │       │
│       │ • 计算 step_cost                 │       │
│       │   (直线=1.0, 对角≈1.414)         │       │
│       │ • new_dist = current.g + step_cost│       │
│       │ • if new_dist ≥ dist[邻居] → 跳过 │       │
│       │   (已有更短路径)                  │       │
│       │ • 更新 dist, 创建邻居节点          │       │
│       │   (h=0, f=new_dist)              │       │
│       │ • open_set.push(邻居)             │       │
│       └──────────────────────────────────┘       │
│  }                                               │
│                                                 │
│  若 open_set 为空仍未到达目标 → 返回失败           │
└────────────────────┬────────────────────────────┘
                     ▼
┌─────────────────────────────────────────────────┐
│          Step 4: 路径回溯                         │
│                                                 │
│  从目标节点开始，沿着 parent_ 数组向前追溯          │
│  直到遇到父节点为 {-1, -1}（起点）                 │
│                                                 │
│  将回溯得到的节点序列反转 → 得到从起点到终点的路径    │
└────────────────────┬────────────────────────────┘
                     ▼
┌─────────────────────────────────────────────────┐
│          Step 5: 统计输出 & 返回                   │
│   - 记录规划耗时、访问节点数、路径长度等             │
│   - 通过 ROS 日志输出 (INFO / WARN)               │
│   - 返回 true (成功) 或 false (失败)               │
└─────────────────────────────────────────────────┘
```

### Dijkstra 主算法核心代码

```cpp
// Dijkstra 与 A* 的核心区别如下（位于 makePlan 主循环中）：

// A*: 优先队列按 f = g + h 排序，h 由启发函数计算
// Dijkstra: 优先队列按 f = g 排序，h ≡ 0

// 起点初始化（Dijkstra 版本）
Node start_node(start_gx, start_gy);
start_node.g = 0.0;
start_node.h = 0.0;       // 无启发函数
start_node.f = start_node.g; // f = g

// 邻居扩展（Dijkstra 版本）
Node neighbor(nx, ny);
neighbor.g = new_dist;
neighbor.h = 0.0;          // 无启发函数
neighbor.f = neighbor.g;   // f = g
```

## 规划器切换

节点 `global_planner_node` 通过 ROS 参数 `planner_name` 在运行时选择规划器：

```bash
# 使用 A* 算法（默认）
roslaunch global_planner_learning planner.launch planner_name:=AStar

# 使用 Dijkstra 算法
roslaunch global_planner_learning planner.launch planner_name:=Dijkstra

# 使用 RRT 算法
roslaunch global_planner_learning planner.launch planner_name:=RRT
```

切换逻辑位于 `mapCallback` 中：

```cpp
if (planner_name_ == "AStar") {
  planner_ = new AStarPlanner(map_, allow_unknown_, use_8_connectivity_);
} else if (planner_name_ == "Dijkstra") {
  planner_ = new DijkstraPlanner(map_, allow_unknown_, use_8_connectivity_);
} else if (planner_name_ == "RRT") {
  double goal_bias = 0.1;
  double step_size = 0.5;
  int max_iter = 5000;
  private_nh_.param("rrt_goal_bias", goal_bias, 0.1);
  private_nh_.param("rrt_step_size", step_size, 0.5);
  private_nh_.param("rrt_max_iter", max_iter, 5000);
  planner_ = new RRTPlanner(map_, allow_unknown_,
                            goal_bias, step_size, max_iter);
} else {
  // fallback to AStar
}
```

RRT 专属参数可通过 launch 文件配置：

```xml
<param name="planner_name" value="RRT"/>
<param name="rrt_goal_bias" value="0.15"/>   <!-- 目标偏置概率 -->
<param name="rrt_step_size" value="0.8"/>     <!-- 扩展步长（米） -->
<param name="rrt_max_iter" value="10000"/>    <!-- 最大迭代次数 -->
```

## RRT 算法

### 概述

**RRT（Rapidly-exploring Random Tree，快速随机探索树）** 是一种基于采样的路径规划算法，由 Steven M. LaValle 于 1998 年提出。与 A\* 和 Dijkstra 等基于栅格搜索的算法不同，RRT 通过在连续空间中随机采样来构建树状结构，特别适合**高维空间**和**复杂约束**下的路径规划问题。

> RRT 属于**概率完备**算法——当采样点趋于无穷时，找到可行路径的概率趋近于 1。但它**不保证最优性**。

### 基本 RRT 算法流程

```
┌─────────────────────────────────────────────────┐
│   输入：起点 start, 终点 goal, 地图/空间信息      │
│   参数：最大迭代次数 K, 步长 step_size            │
└────────────────────┬────────────────────────────┘
                     ▼
┌─────────────────────────────────────────────────┐
│     Step 1: 初始化                                │
│     T.add(start)   ← 以起点为根节点               │
└────────────────────┬────────────────────────────┘
                     ▼
┌─────────────────────────────────────────────────┐
│     Step 2: 主循环 (k = 1 → K)                   │
│                                                 │
│    ① x_rand = sample()                          │
│       ← 在自由空间中随机采样一个点                │
│                                                 │
│    ② x_near = nearest(T, x_rand)                │
│       ← 在树 T 中找到离 x_rand 最近的节点         │
│                                                 │
│    ③ x_new = steer(x_near, x_rand, step_size)   │
│       ← 从 x_near 向 x_rand 方向步进 step_size   │
│                                                 │
│    ④ if collision_free(x_near, x_new)           │
│         T.add_node(x_new)                       │
│         T.add_edge(x_near, x_new)               │
│       ← 若无碰撞则加入新节点和边                 │
│                                                 │
│    ⑤ if dist(x_new, goal) < threshold           │
│         → 连接目标点，返回路径                   │
│                                                 │
└────────────────────┬────────────────────────────┘
                     ▼
┌─────────────────────────────────────────────────┐
│     Step 3: 路径回溯                              │
│     从目标点/最近节点沿父节点链回溯至起点          │
│     若采样结束仍未到达 → 返回失败                  │
└─────────────────────────────────────────────────┘
```

### RRT 核心步骤详解

#### 1. 随机采样 `sample()`

在无障碍的构型空间（C-space）中均匀随机采样。采样点可以是：

- **均匀采样**：全空间等概率采样，简单但效率低
- **偏置采样**（Biased Sampling）：以一定概率直接采样目标点，加速收敛

```cpp
Node sample() {
  if (rand() / RAND_MAX < goal_bias) {
    return goal_node; // 以 goal_bias 概率直接选择目标
  }
  // 否则在空间范围内均匀随机采样
  double x = min_x + (rand() / RAND_MAX) * (max_x - min_x);
  double y = min_y + (rand() / RAND_MAX) * (max_y - min_y);
  return Node(x, y);
}
```

#### 2. 最近邻查找 `nearest(T, x_rand)`

遍历当前树中所有节点，找到与采样点欧氏距离最近的节点：

```
x_near = argmin_{v ∈ T} ||v - x_rand||
```

当树中节点数量很大时，可使用 KD-Tree 等空间索引结构加速最近邻搜索。

#### 3. 扩展步进 `steer(x_near, x_rand, step_size)`

从最近节点向采样点方向移动固定步长，生成新节点：

```
dir = normalize(x_rand - x_near)
x_new = x_near + step_size * dir
```

步长 `step_size` 是算法的重要参数：
- **步长过大**：容易越过狭窄通道，路径粗糙
- **步长过小**：树增长缓慢，收敛速度降低

#### 4. 碰撞检测 `collision_free(x_near, x_new)`

检查从 `x_near` 到 `x_new` 的线段是否与障碍物相交。在栅格地图中，可通过 Bresenham 线段算法遍历路径上的栅格，检查每个栅格是否被占用。

### 基本 RRT 的局限性

| 问题 | 说明 |
|------|------|
| **非最优** | 找到的路径通常不是最短路径，路径往往曲折 |
| **非渐近最优** | 即使无限运行，也不保证收敛到最优解 |
| **狭窄通道敏感** | 在狭窄通道中采样概率低，难以找到路径 |
| **路径不平滑** | 由随机采样点直接连接而成，转折突兀 |

---

## RRT 改进算法

### 1. RRT\*（RRT-Star）— 渐近最优版本

RRT\* 是 RRT 最重要的改进，由 Sertac Karaman 和 Emilio Frazzoli 于 2010 年提出。在基本 RRT 的基础上增加了**重连接（Rewiring）**步骤，保证了**渐近最优性**（asymptotic optimality）。

#### 与基本 RRT 的区别

```
标准 RRT:  sample → nearest → steer → insert
RRT*:     sample → nearest → steer → insert → rewire
```

#### 重连接（Rewiring）过程

```
Step 1: 插入新节点 x_new
Step 2: 在以 x_new 为中心、半径 r 的邻域内查找候选节点
        X_near = { x ∈ T : ||x - x_new|| < r }
Step 3: 选择最优父节点
        x_min = argmin_{x ∈ X_near} cost(x) + dist(x, x_new)
        将 x_new 的父节点改为 x_min（若更优）
Step 4: 重连接邻域节点
        for each x ∈ X_near:
          if cost(x_new) + dist(x_new, x) < cost(x):
            将 x 的父节点改为 x_new（更新代价）
```

#### 关键参数

| 参数 | 说明 |
|------|------|
| 搜索半径 `r` | 重连接时查找邻居的范围，通常与节点数呈反比：`r = γ (log(n)/n)^(1/d)` |
| 常数 `γ` | 影响收敛速度的理论常数 |

#### 伪代码

```
function RRT*(start, goal):
  T.init(start)
  for k = 1 to K:
    x_rand = sample()
    x_near = nearest(T, x_rand)
    x_new = steer(x_near, x_rand, step_size)
    if collision_free(x_near, x_new):
      X_near = near_nodes(T, x_new, r)       // 查找邻域节点
      x_min = choose_parent(X_near, x_new)   // 选择最优父节点
      T.add_node(x_new)
      T.add_edge(x_min, x_new)
      T.rewire(X_near, x_new)                // 重连接邻域
    if dist(x_new, goal) < threshold:
      return extract_path(T, x_new)
  return failure
```

### 2. Informed RRT\* —  Informed 采样优化

Informed RRT\* 在 RRT\* 的基础上，通过限制采样区域来提高收敛速度。

#### 核心思想

一旦找到初始可行路径，后续采样范围被限制在一个**椭圆区域**内：

- 椭圆的两个焦点：起点 `x_start` 和终点 `x_goal`
- 椭圆的长轴长度 = 当前最优路径长度 `c_best`
- 椭圆的焦距 `c_min = ||x_goal - x_start||`

椭圆内任意点 `x` 满足：

```
||x - x_start|| + ||x - x_goal|| ≤ c_best
```

任何比当前路径短的路径，其上的所有点必定落在这个椭圆内部。因此，将采样限制在椭圆内可大幅提升搜索效率，使算法更快收敛到最优解。

```
         ┌─── 椭圆边界 ───┐
          \             /
           \    ┌───────┐
            \   │ 采样域 │
             \  │   x    │
              \ │   x    │
    x_start ───●─────────●─── x_goal
              / │   x    │
             /  │   x    │
            /   └───────┘
           /   采样区域
          /  随 c_best 缩小
         └─── 渐近收敛 ───┘
```

随着 `c_best` 不断接近最优值 `c_min`，椭圆逐渐收缩，采样越来越集中，加速收敛。

### 3. RRT-Connect（双树 RRT）

RRT-Connect 同时从起点和终点生长两棵树，加速收敛。

```
T_a: 从起点生长
T_b: 从终点生长

每次迭代：
  ① T_a 正常扩展一个节点
  ② T_b 尝试向 T_a 的新节点方向**持续步进**
     （connect 阶段，不采样，连续步进直到到达或碰撞）
  ③ 交替角色：下次交换两棵树的身份
```

**Connect 扩展**比普通 RRT 的步进更激进——它连续步进直到到达目标节点或发生碰撞，使得两棵树能快速连接。

### 4. Bi-RRT（Bidirectional RRT）

Bi-RRT 与 RRT-Connect 类似，也使用双树结构。区别在于 Bi-RRT 每次迭代两棵树各自扩展一个节点，然后尝试连接。

```
每轮迭代：
  ① T_a 扩展一步（标准 RRT 步骤）
  ② T_b 扩展一步（标准 RRT 步骤）
  ③ 尝试连接 T_a 和 T_b 的最新节点
  ④ 交换 T_a 与 T_b
```

### 5. Anytime RRT\*（任意时间 RRT\*）

Anytime RRT\* 在有限时间内先生成一个初步可行路径，然后利用剩余时间持续优化：

```
① 快速找到一条可行路径（作为初始解）
② 继续采样和重连接，不断改进路径质量
③ 随时可中断，返回当前最优路径
```

该特性使得 Anytime RRT\* 非常适合于**实时规划**场景（如机器人运动规划）。

---

## 三种算法范式对比

| 维度 | A* / Dijkstra | RRT | RRT* / Informed RRT* |
|------|---------------|-----|---------------------|
| **搜索方式** | 确定性搜索（图搜索） | 随机采样（概率搜索） | 随机采样 + 优化 |
| **空间表示** | 离散栅格（需栅格化） | 连续空间 | 连续空间 |
| **最优性** | ✅ 保证最优 | ❌ 不保证最优 | ✅ 渐近最优 |
| **完备性** | ✅ 分辨率完备 | ✅ 概率完备 | ✅ 概率完备 |
| **高维适用性** | ❌ 维度灾难 | ✅ 适合高维 | ✅ 适合高维 |
| **狭窄通道** | ✅ 可以处理 | ⚠️ 困难 | ⚠️ 仍困难 |
| **路径质量** | 平滑（可优化） | 粗糙、锯齿状 | 渐近优化 |
| **实现复杂度** | 中等 | 低 | 较高 |
| **内存开销** | O(地图大小) | O(节点数) | O(节点数) |

### 选择指南

| 应用场景 | 推荐算法 |
|----------|----------|
| 二维栅格地图，已知静态环境 | **A\***（最优、高效） |
| 需要最短路径保证 | **A\*** 或 **Dijkstra** |
| 高维空间（机械臂 >6 DOF） | **RRT** 或 **RRT-Connect** |
| 追求路径最优，不要求实时 | **RRT\*** 或 **Informed RRT\*** |
| 实时规划，可接受次优解 | **Anytime RRT\*** |
| 狭窄通道环境 | **RRT-Connect**（双树） |
| 无启发信息的通用图 | **Dijkstra** |

## 算法对比总结

| 特性 | A* | Dijkstra | RRT | RRT\* |
|------|-----|----------|-----|-------|
| 搜索策略 | 启发式图搜索 | 广度优先（加权） | 随机采样 | 随机采样 + 重连接 |
| 排序/选择依据 | `f = g + h` | `f = g` | 随机采样 + 最近邻 | 随机采样 + 代价优化 |
| 方向引导 | 有（启发式） | 无 | 弱（可加偏置） | 弱（可加偏置） |
| 最优性 | ✅ 保证 | ✅ 保证 | ❌ 不保证 | ✅ 渐近最优 |
| 高维扩展性 | ❌ 差 | ❌ 差 | ✅ 好 | ✅ 好 |
| 实现文件 | `astar_planner.cpp` | `dijkstra_planner.cpp` | `rrt_planner.cpp` | — |

## 依赖

- ROS (Melodic / Noetic)
- `nav_msgs` / `geometry_msgs`
- C++14 / C++17
