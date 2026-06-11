# ROS 地图生成与加载指南

## 你的工作空间已有的资源

| 资源类型 | 路径 | 说明 |
|---------|------|------|
| Gazebo 仿真世界 | `src/sim_env/worlds/` | warehouse, workshop, museum 等 10+ 个世界 |
| 预生成地图 | `src/sim_env/maps/` | warehouse/ 和 workshop/ 两个已有地图 |
| 机器人模型 | `src/sim_env/urdf/` | nanocar, turtlebot3 系列 |
| 导航配置 | `src/sim_env/config/` | costmap, amcl, move_base 参数 |

---

## 方法一：使用已有的地图（推荐，快速开始）

你的 workspace 已经有两张现成的地图，可以直接用于全局路径规划。

### warehouse 地图

```yaml
# src/sim_env/maps/warehouse/warehouse.yaml
image: ./warehouse.pgm     # 地图图片文件
resolution: 0.05           # 每个像素 0.05 米（5cm）
origin: [-12.0, -12.0, 0.0] # 地图左下角在现实世界中的坐标 (x, y, yaw)
negate: 0                  # 是否反转黑白
occupied_thresh: 0.65      # 像素值 > 0.65 视为障碍物
free_thresh: 0.196         # 像素值 < 0.196 视为自由空间
```

### 启动方式

```bash
# 1. 启动 Gazebo 仿真（带 warehouse 世界）
roslaunch sim_env main.launch world_parameter:=warehouse

# 或手动启动 map_server 加载地图（不启动 Gazebo）
rosrun map_server map_server src/sim_env/maps/warehouse/warehouse.yaml
```

---

## 方法二：使用 gmapping SLAM 生成新地图

如果你想在**自己的环境中**生成新地图，流程如下：

### 第1步：启动 Gazebo 仿真

```bash
roslaunch sim_env main.launch world_parameter:=warehouse
```

这会启动：
- Gazebo 仿真器 + warehouse 世界
- 机器人模型
- `map_server` 加载已有地图（但做 SLAM 时通常不需要加载旧地图）

### 第2步：启动 gmapping SLAM 节点

```bash
rosrun gmapping slam_gmapping scan:=scan
```

`gmapping` 会订阅激光雷达数据（`/scan`）和里程计数据（`/odom`），实时构建地图。

### 第3步：控制机器人探索环境

通过键盘控制机器人在环境中移动，覆盖所有区域：

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

### 第4步：保存生成的地图

当地图构建完成后（即机器人已遍历所有区域）：

```bash
# 将当前地图保存到文件
rosrun map_server map_saver -f my_map
```

这会生成两个文件：
- `my_map.pgm` — 地图图片（灰度图）
- `my_map.yaml` — 地图元数据文件

### 第5步：将地图放到功能包中

```bash
# 创建地图目录
mkdir -p src/global_planner_learning/maps/my_map

# 复制地图文件
cp my_map.pgm my_map.yaml src/global_planner_learning/maps/my_map/
```

---

## 方法三：使用其他 SLAM 方案

### 1. Google Cartographer（推荐，精度高）

```bash
# 安装
sudo apt install ros-noetic-cartographer ros-noetic-cartographer-ros

# 启动（需要自行编写配置文件）
roslaunch cartographer_ros demo.launch
```

### 2. hector_slam（不需要里程计）

```bash
# 安装
sudo apt install ros-noetic-hector-slam

# 启动
roslaunch hector_slam hector_slam.launch
```

### 3. RTAB-Map（RGB-D SLAM）

```bash
# 安装
sudo apt install ros-noetic-rtabmap-ros

# 启动
roslaunch rtabmap_ros rtabmap.launch
```

---

## 加载地图用于全局路径规划

地图准备好后，通过 `map_server` 节点加载，然后由 Navigation Stack 使用。

### 方式1：在 launch 文件中加载

```xml
<!-- 加载地图 -->
<node name="map_server" pkg="map_server" type="map_server"
      args="$(find your_package)/maps/my_map/my_map.yaml" />
```

### 方式2：通过命令行加载

```bash
rosrun map_server map_server path/to/your_map.yaml
```

### 地图发布后验证

```bash
# 查看地图话题
rostopic echo /map

# 在 rviz 中查看
# 添加 → Map → Topic: /map
```

---

## 地图加载后的完整导航流程

```
Gazebo (仿真环境)
    │
    ├── 机器人传感器数据 (scan, odom, tf)
    │
    ├── map_server → /map 话题（静态地图）
    │
    ├── amcl → /amcl_pose（定位）
    │
    └── move_base
         ├── global_costmap（全局代价地图 = 静态地图 + 障碍物层 + 膨胀层）
         ├── global_planner → /plan（全局路径）
         ├── local_costmap（局部代价地图 = 滚动窗口）
         └── local_planner → /cmd_vel（速度指令）
```

---

## 地图文件格式详解

### .pgm 文件（可移植灰度图）

- **白色像素（255）**：自由空间，机器人可通行
- **黑色像素（0）**：障碍物，不可通行
- **灰色像素（205）**：未知区域，未探索
- 灰度值介于 free_thresh 和 occupied_thresh 之间的像素为未知区域

### .yaml 文件参数

| 参数 | 说明 |
|------|------|
| `image` | 地图图片路径（相对或绝对） |
| `resolution` | 每个像素对应的实际距离（米/像素） |
| `origin` | 地图左下角的真实世界坐标 `[x, y, yaw]` |
| `negate` | 0=白色自由/黑色障碍，1=相反 |
| `occupied_thresh` | 像素值超过此阈值视为占据 |
| `free_thresh` | 像素值低于此阈值视为自由 |
| `mode` | `trinary`（默认）/ `scale` / `raw` |

---

## 常见问题

### Q: 加载地图时 map_server 报错？

确保 `.yaml` 中 `image` 路径正确（可以是绝对路径或相对于 `.yaml` 文件的路径）。

### Q: 地图加载后 rviz 中不显示？

检查：
1. `map_server` 节点是否在运行（`rosnode list | grep map_server`）
2. 话题 `/map` 是否有数据（`rostopic echo /map | head`）
3. rviz 中 Fixed Frame 是否设为 `map`

### Q: gmapping 构建的地图不完整？

机器人没有遍历所有区域。控制机器人去未探索的区域。
