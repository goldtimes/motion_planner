# ROS move_base 导航流程详解

> 参考代码：本项目的 `sim_env/launch/include/navigation/move_base.launch.xml`
> 及 ROS 官方 `move_base` 源码

---

## 一、整体架构

```
                        ┌─────────────────────────┐
                        │        RViz              │
                        │  (2D Nav Goal 点击)      │
                        └──────────┬──────────────┘
                                   │ /move_base_simple/goal
                                   ▼
                   ┌───────────────────────────────┐
                   │          move_base             │
                   │  (核心导航调度器)               │
                   │                               │
                   │  ┌─────────┐  ┌─────────────┐ │
                   │  │全局规划器 │  │ 局部规划器   │ │
                   │  │PathPlanner│  │DWA/TEB/PID..│ │
                   │  └────┬────┘  └──────┬──────┘ │
                   │       │              │         │
                   │  ┌────▼────┐  ┌──────▼──────┐ │
                   │  │全局代价图 │  │ 局部代价图   │ │
                   │  │costmap   │  │ costmap     │ │
                   │  └─────────┘  └─────────────┘ │
                   └───────────────┬───────────────┘
                                   │ /cmd_vel
                                   ▼
                          ┌─────────────────┐
                          │  机器人底层控制器  │
                          │  (差速/全向轮)    │
                          └─────────────────┘
```

---

## 二、启动流程

### 2.1 启动文件链

以 `main.sh` → `main.launch` → `config.launch` 为例：

```
main.sh
  └─ roslaunch sim_env main.launch
       └─ config.launch
            ├─ Gazebo (仿真环境)
            ├─ map_server (地图)
            ├─ start_robots.launch.xml
            │    ├─ 机器人模型(spawn)
            │    └─ move_base.launch.xml
            │         ├─ 全局规划器插件
            │         ├─ 局部规划器插件
            │         ├─ 全局代价图参数
            │         ├─ 局部代价图参数
            │         └─ move_base 通用参数
            └─ RViz (可视化)
```

### 2.2 插件加载机制

move_base 使用 `pluginlib` 动态加载规划器：

```xml
<!-- move_base.launch.xml -->
<param name="base_global_planner" value="path_planner/PathPlanner" />
<param name="base_local_planner" value="dwa_controller/DWAController"
       if="$(eval arg('local_planner')=='dwa')" />
```

每个规划器都是一个插件，通过 `pluginlib` 导出：

```cpp
// my_dwa_controller.cpp
PLUGINLIB_EXPORT_CLASS(rmp::controller::MyDWAController, nav_core::BaseLocalPlanner)
```

配套的 `plugin.xml` 注册插件：
```xml
<library path="lib/libmy_dwa_controller">
    <class name="my_dwa_controller/MyDWAController"
           type="rmp::controller::MyDWAController"
           base_class_type="nav_core::BaseLocalPlanner" />
</library>
```

---

## 三、导航主循环

move_base 的核心是一个**状态机**，每帧执行以下流程：

### 3.1 接收目标

```
用户点击 RViz "2D Nav Goal"
    ↓
/move_base_simple/goal (PoseStamped)
    ↓
move_base::executeCycle()
    ↓
调用全局规划器 setPlan() 触发重新规划
```

### 3.2 全局规划

```
全局规划器 (如 A*)
    ↓ 输入：起点(机器人位姿) + 终点(用户点击) + 全局代价图
    ↓ 输出：全局路径 (std::vector<PoseStamped>)
    ↓
move_base 保存全局路径
    ↓
调用局部规划器 setPlan(global_plan)
```

### 3.3 控制循环 (核心)

move_base 以 `controller_frequency` (默认 10Hz) 运行以下循环：

```
while(未到达目标 && 未超时) {
    ┌─────────────────────────────────────────────┐
    │ 1. 获取当前位姿 (来自 TF + 里程计)           │
    │ 2. 更新局部代价图 (激光雷达 → costmap)       │
    │                                              │
    │ 3. isGoalReached()?                          │
    │    ├─ true  → 停止, 标记完成                  │
    │    └─ false → 继续                           │
    │                                              │
    │ 4. 获取当前机器人附近的路径段                   │
    │    (从全局路径中截取局部窗口)                   │
    │                                              │
    │ 5. computeVelocityCommands(cmd_vel)          │
    │    ├─ 局部规划器计算速度指令                   │
    │    ├─ true  → 发布 cmd_vel 到机器人           │
    │    └─ false → 重试(累计 patience)             │
    │                                              │
    │ 6. 发布可视化 (全局/局部路径, 代价图等)        │
    └─────────────────────────────────────────────┘
}
```

### 3.4 局部计划裁剪

每个周期，move_base 从全局路径中截取机器人附近的一段作为**局部参考路径**：

```
全局路径: [w0]→[w1]→[w2]→[w3]→[w4]→...→[wn]
                         ↑
                      机器人位置
                         ↓
局部参考路径:           [w2]→[w3]→[w4]→...→[wn]
                      (从最近点开始到终点)
```

在我们的代码中，这由 `prunePlan()` (TEB) 或 `planner_util_.getLocalPlan()` (DWA) 实现。

---

## 四、代价图系统 (costmap)

### 4.1 双层代价图

```
全局代价图 (global_costmap)
  ├─ 坐标系: map
  ├─ 大小: 整个地图
  ├─ 更新频率: 低 (1~5Hz)
  ├─ 用途: 全局路径规划
  └─ 图层: 静态地图层 + 障碍物层(可选)

局部代价图 (local_costmap)
  ├─ 坐标系: odom
  ├─ 大小: 机器人周围窗口 (4×4m 等)
  ├─ 更新频率: 高 (10Hz)
  ├─ 用途: 局部路径规划/避障
  └─ 图层: 障碍物层(激光) + 膨胀层
```

### 4.2 代价值含义

```
255 (NO_INFORMATION)     → 未知区域
254 (LETHAL_OBSTACLE)    → 障碍物占据
253 (INSCRIBED_INFLATED) → 机器人足迹触碰障碍
252 → 1                  → 膨胀区域 (越靠近障碍数值越大)
0  (FREE_SPACE)          → 自由空间
```

---

## 五、完整数据流（以 DWA 为例）

```
┌──────┐    /map    ┌───────────┐
│map_  │──────────→│全局代价图  │←──── global_plan
│server│           └───────────┘
└──────┘                   
                           
┌──────┐    /scan   ┌───────────┐
│激光雷达│──────────→│局部代价图  │
└──────┘           │(障碍层+    │
                   │ 膨胀层)    │
┌────────┐  /odom   └─────┬─────┘
│里程计  │─────────→      │
└────────┘         TF     │
                   │      │
                   ▼      ▼
            ┌──────────────────┐
            │   move_base       │
            │                   │
            │  1. getRobotPose()│ ← 从 TF 获取
            │  2. getLocalPlan()│ ← 从全局路径截取
            │  3. computeVel()  │ ← 调用局部规划器
            │  4. publish cmd_vel│
            └──────────────────┘
```

### DWA 局部规划器内部流程（以我们写的 my_dwa_controller 为例）

```
computeVelocityCommands()
  │
  ├─ 获取机器人位姿 (x, y, θ)
  ├─ 获取当前速度 (vx, vy, vth)
  ├─ 计算动态窗口 (速度限 ∩ 加速度限)
  ├─ 采样速度 (v, ω) 网格
  │
  ├─ [对每个采样] ──────────────────────┐
  │   ├─ 预测轨迹 (unicycle模型, 3秒)    │
  │   └─ 评估代价                       │
  │        ├─ 障碍物代价 (costmap查表)    │
  │        ├─ 路径对齐代价 (到全局路径距离) │
  │        ├─ 目标距离代价                │
  │        └─ 速度鼓励代价               │
  │                                     │
  ├─ 选取最优轨迹 ◄─────────────────────┘
  ├─ 输出 cmd_vel (v, ω)
  └─ 发布局部路径可视化
```

---

## 六、失败处理机制

move_base 有 patience（耐心）机制：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `planner_frequency` | 0.0 Hz | 全局重规划频率 (0=仅开始时规划) |
| `planner_patience` | 5.0 s | 全局规划失败重试多久 |
| `controller_frequency` | 10.0 Hz | 局部规划频率 |
| `controller_patience` | 15.0 s | 局部规划失败重试多久 |
| `oscillation_timeout` | 10.0 s | 检测到震荡时等待多久后放弃 |
| `oscillation_distance` | 0.2 m | 震荡检测距离阈值 |

### 6.1 失败流程

```
computeVelocityCommands() 返回 false
    ↓
move_base 递增 controller_patience 计数器
    ↓
┌─ 超过 controller_patience (15s)?
│   ├─ 是 → 放弃当前目标, 通知用户
│   └─ 否 → 继续重试
│
└─ 检测到震荡 (来回摆动 > oscillation_distance)?
    ├─ 是 → 清除代价图, 强制重规划
    └─ 否 → 继续当前循环
```

---

## 七、在你的项目中的对应关系

| ROS 概念 | 本项目中的实现 |
|----------|--------------|
| 全局规划器接口 | `nav_core::BaseGlobalPlanner` |
| 全局规划器实现 | `path_planner` (A*, JPS, RRT, etc.) |
| 局部规划器接口 | `nav_core::BaseLocalPlanner` |
| 局部规划器实现 | `dwa_controller`, `my_dwa_controller`, `teb_controller`, `pid_controller` 等 |
| 启动配置 | `sim_env/launch/include/navigation/move_base.launch.xml` |
| 用户配置 | `user_config/user_config.yaml` (选择用什么规划器) |
| 全局代价图参数 | `sim_env/config/robots/*/global_costmap_params_*.yaml` |
| 局部代价图参数 | `sim_env/config/robots/*/local_costmap_params_*.yaml` |
| move_base 参数 | `sim_env/config/move_base_params.yaml` |
| 仿真环境 | Gazebo + `sim_env/worlds/` |

---

## 八、调试技巧

### 查看 move_base 状态

```bash
# 查看 move_base 是否在正常工作
rostopic echo /move_base/status

# 查看当前是否到达目标
rostopic echo /move_base/result

# 查看规划频率
rostopic hz /cmd_vel
```

### RViz 中关注的 Topic

| 话题 | 类型 | 说明 |
|------|------|------|
| `/move_base/MyDWAController/local_plan` | Path | 局部规划路径 |
| `/move_base/MyDWAController/global_plan` | Path | 全局规划路径 |
| `/move_base/MyDWAController/explored_trajectories` | MarkerArray | 所有采样的轨迹 (绿=有效, 红=无效) |
| `/move_base/current_goal` | PoseStamped | 当前目标点 |
| `/cmd_vel` | Twist | 输出到机器人的速度指令 |
