# ROS 全局路径规划学习路线图

## 概述

全局路径规划（Global Path Planning）是 ROS Navigation Stack 的核心组成部分，负责在已知的静态地图上规划出一条从起点到目标点的最优（或可行）路径。本功能包旨在帮助你系统性地学习这一领域的知识与实践。

---

## 学习阶段

### 阶段一：基础概念（理论）

1. **ROS Navigation Stack 架构**
   - `move_base` —— 导航行为控制器
   - `global_planner` —— 全局路径规划器
   - `local_planner` —— 局部路径规划器（轨迹跟踪与避障）
   - `costmap_2d` —— 代价地图（全局/局部）
   - `amcl` —— 自适应蒙特卡洛定位
   - 参考：[ROS Navigation Wiki](http://wiki.ros.org/navigation)

2. **全局路径规划的核心接口**
   - `nav_core::BaseGlobalPlanner` —— 所有全局规划器必须继承的基类
   - `makePlan(const Pose& start, const Pose& goal, vector<PoseStamped>& plan)` —— 核心规划方法
   - `initialize(string name, Costmap2DROS* costmap)` —— 初始化方法
   - `pluginlib` 插件机制 —— 如何将规划器注册为可插拔组件

3. **经典全局路径规划算法**
   - **Dijkstra**（最短路径，BFS 加权版）
     - 实现：ROS `navfn` 包（`NavFnROS`）
     - 特点：保证最优解，但效率较低
   - **A\***（启发式搜索）
     - 实现：ROS `global_planner` 包（`GlobalPlanner`，默认使用 A*）
     - 特点：引入启发函数（欧氏距离/曼哈顿距离），效率高于 Dijkstra
   - **D\* / D\* Lite**（动态环境重规划）
     - ROS 中较少直接使用，但思想重要
   - **RRT / RRT\***（基于采样的规划）
     - 适合高维空间或非完整约束

4. **代价地图（Costmap2D）**
   - 静态地图层（Static Layer）—— SLAM 构建的地图
   - 障碍物层（Obstacle Layer）—— 传感器实时检测的障碍
   - 膨胀层（Inflation Layer）—— 障碍物膨胀，保证机器人不会碰撞
   - 主图层（Master Layer）—— 各层叠加后的最终代价图

### 阶段二：动手实践（代码）

> 后续将在 `src/` 目录下逐步实现以下内容。

1. **最简单的全局规划器** —— 实现 `BaseGlobalPlanner` 接口
   - 直连规划：起点到终点画一条直线
   - 注册为 pluginlib 插件
   - 在 rviz 中可视化测试

2. **基于栅格搜索的规划器**
   - Dijkstra 算法实现
   - A\* 算法实现
   - 代价地图读取与障碍物检测

3. **集成到 Navigation Stack**
   - 编写 `move_base` 启动文件 （`launch/`）
   - 配置代价地图参数 （`config/`）
   - 在仿真环境中测试（Stage / Gazebo）

### 阶段三：进阶学习

- **Hybrid A\*** —— 考虑运动学约束的搜索算法
- **State Lattice Planning** —— 基于状态格子的规划
- **TEB / DWA** 局部规划器如何与全局规划配合
- **ROS2 Navigation2** —— `nav2_planner` 体系

---

## 推荐资源

| 资源 | 链接 |
|------|------|
| ROS Navigation Tutorial | http://wiki.ros.org/navigation/Tutorials |
| ROS Navigation 源码 | https://github.com/ros-planning/navigation |
| 代价地图配置 | http://wiki.ros.org/costmap_2d |
| A* 算法可视化 | https://qiao.github.io/PathFinding.js/visual/ |
| 路径规划综述 | https://arxiv.org/abs/1902.03446 |

---

## 学习检查清单

- [ ] 理解 Navigation Stack 的整体架构
- [ ] 阅读 `nav_core::BaseGlobalPlanner` 接口源码
- [ ] 理解 pluginlib 插件机制
- [ ] 能够在 rviz 中运行 move_base 并切换全局规划器
- [ ] 理解代价地图的图层构成
- [ ] 动手实现一个自己的全局规划器
- [ ] 在仿真环境中验证规划效果
