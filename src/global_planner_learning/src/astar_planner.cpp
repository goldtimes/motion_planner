/**
 * @file astar_planner.cpp
 * @brief A* 路径规划核心算法实现
 * @author learner
 * @version 0.1
 */

#include "global_planner_learning/astar_planner.h"
#include <ros/ros.h>

namespace global_planner_learning {

// ==================== 静态常量定义 ====================
constexpr int AStarPlanner::DX4_[4];
constexpr int AStarPlanner::DY4_[4];
constexpr int AStarPlanner::DX8_[8];
constexpr int AStarPlanner::DY8_[8];

// ==================== 构造函数 ====================
AStarPlanner::AStarPlanner(const nav_msgs::OccupancyGrid::ConstPtr &map,
                           bool allow_unknown, bool use_8_connectivity)
    : map_(map.get()), allow_unknown_(allow_unknown),
      use_8_connectivity_(use_8_connectivity) {
  width_ = map->info.width;
  height_ = map->info.height;
  resolution_ = map->info.resolution;
  origin_x_ = map->info.origin.position.x;
  origin_y_ = map->info.origin.position.y;
}

// ==================== 坐标转换 ====================
bool AStarPlanner::worldToGrid(double wx, double wy, int &gx, int &gy) const {
  gx = static_cast<int>((wx - origin_x_) / resolution_);
  gy = static_cast<int>((wy - origin_y_) / resolution_);
  return (gx >= 0 && gx < width_ && gy >= 0 && gy < height_);
}

void AStarPlanner::gridToWorld(int gx, int gy, double &wx, double &wy) const {
  wx = origin_x_ + (gx + 0.5) * resolution_;
  wy = origin_y_ + (gy + 0.5) * resolution_;
}

// ==================== 可通行性检查 ====================
bool AStarPlanner::isWalkable(int gx, int gy) const {
  // 边界检查
  if (gx < 0 || gx >= width_ || gy < 0 || gy >= height_)
    return false;

  int8_t cost = map_->data[gy * width_ + gx];

  // 未知区域
  if (cost == UNKNOWN_VALUE)
    return allow_unknown_;

  // 障碍物
  if (cost >= OCCUPIED_THRESH)
    return false;

  return true;
}

// ==================== 启发函数 ====================
double AStarPlanner::getHeuristic(int gx, int gy, int goal_x,
                                  int goal_y) const {
  double dx = static_cast<double>(goal_x - gx);
  double dy = static_cast<double>(goal_y - gy);

  if (use_8_connectivity_) {
    // 对角线距离（允许 8 方向移动）
    return std::max(std::abs(dx), std::abs(dy));
  } else {
    // 曼哈顿距离（只允许 4 方向移动）
    return std::abs(dx) + std::abs(dy);
  }
}

// ==================== 路径回溯 ====================
void AStarPlanner::reconstructPath(
    int goal_x, int goal_y, std::vector<geometry_msgs::PoseStamped> &plan) {
  // 用一个 map 存储父节点关系（路径回溯）
  // 为了提高效率，我们使用一个二维数组来记录每个节点的父节点
  // 但由于地图可能很大，这里我们在 visited_nodes_ 中查找
  // 更高效的方式是在搜索时维护一个 parent 数组，但为了清晰起见此处在 visited
  // 中回溯
  ROS_INFO("[A*] reconstruct path from goal: (%d, %d)", goal_x, goal_y);
  // 从目标点开始回溯
  std::vector<geometry_msgs::PoseStamped> path_nodes;
  int cx = goal_x, cy = goal_y;

  // 先通过 visited_nodes_ 重建路径
  // 实际更好的做法是在 A* 搜索时维护 parent 矩阵
  // 但为了代码简洁，这里我们重新搜索 parent 关系
  // 使用一个临时地图来存储 parent
  std::vector<std::pair<int, int>> parent_map(width_ * height_, {-1, -1});

  // 从 visited_nodes_ 中提取 parent 关系
  for (const auto &node : visited_nodes_) {
    int idx = node.y * width_ + node.x;
    parent_map[idx] = {node.parent_x, node.parent_y};
  }

  // 回溯
  while (cx >= 0 && cy >= 0) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = map_->header.frame_id;
    pose.header.stamp = ros::Time(0);
    gridToWorld(cx, cy, pose.pose.position.x, pose.pose.position.y);
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;

    path_nodes.push_back(pose);

    int idx = cy * width_ + cx;
    auto &parent = parent_map[idx];
    if (parent.first < 0 || parent.second < 0)
      break;
    cx = parent.first;
    cy = parent.second;

    // 防止死循环（起点自己指向自己）
    if (cx == goal_x && cy == goal_y)
      break;
  }

  // 反转路径（从起点到终点）
  std::reverse(path_nodes.begin(), path_nodes.end());
  plan = path_nodes;
}

// ==================== A* 主算法 ====================
bool AStarPlanner::makePlan(const geometry_msgs::PoseStamped &start_world,
                            const geometry_msgs::PoseStamped &goal_world,
                            std::vector<geometry_msgs::PoseStamped> &plan) {
  plan.clear();
  visited_nodes_.clear();

  // 1. 世界坐标 → 栅格坐标
  int start_gx, start_gy, goal_gx, goal_gy;
  if (!worldToGrid(start_world.pose.position.x, start_world.pose.position.y,
                   start_gx, start_gy)) {
    ROS_WARN("[A*] start pose out of map range!");
    return false;
  }
  if (!worldToGrid(goal_world.pose.position.x, goal_world.pose.position.y,
                   goal_gx, goal_gy)) {
    ROS_WARN("[A*] goal pose out of map range!");
    return false;
  }

  // 2. 检查起点和终点是否可通行
  if (!isWalkable(start_gx, start_gy)) {
    ROS_WARN("[A*] start pose on obstacle!");
    return false;
  }
  if (!isWalkable(goal_gx, goal_gy)) {
    ROS_WARN("[A*] goal pose on obstacle!");
    return false;
  }

  // 3. 初始化 A* 搜索
  // 使用二维数组标记是否已访问 (closed set)
  std::vector<bool> closed_set(width_ * height_, false);

  // 优先队列 (open set)，最小堆
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_set;

  // 起点
  Node start_node(start_gx, start_gy);
  start_node.h = getHeuristic(start_gx, start_gy, goal_gx, goal_gy);
  start_node.f = start_node.g + start_node.h;
  start_node.parent_x = start_gx;
  start_node.parent_y = start_gy;
  open_set.push(start_node);
  visited_nodes_.push_back(start_node);

  // 选择邻域偏移
  const int *dx = use_8_connectivity_ ? DX8_ : DX4_;
  const int *dy = use_8_connectivity_ ? DY8_ : DX4_;
  int num_neighbors = use_8_connectivity_ ? 8 : 4;

  // 4. A* 主循环
  while (!open_set.empty()) {
    // 取出 f 值最小的节点
    Node current = open_set.top();
    open_set.pop();

    // 如果已经在 closed set 中，跳过
    int idx = current.y * width_ + current.x;
    if (closed_set[idx])
      continue;

    // 标记为已访问
    closed_set[idx] = true;

    // 到达目标点（允许一定容差，精确到栅格即可）
    if (current.x == goal_gx && current.y == goal_gy) {
      reconstructPath(goal_gx, goal_gy, plan);
      ROS_INFO("[A*] plan success! visited nodes: %zu", visited_nodes_.size());

      return true;
    }

    // 遍历邻居
    for (int i = 0; i < num_neighbors; ++i) {
      int nx = current.x + dx[i];
      int ny = current.y + dy[i];

      // 检查边界和可通行性
      if (!isWalkable(nx, ny))
        continue;

      int nidx = ny * width_ + nx;
      if (closed_set[nidx])
        continue;

      // 计算移动代价（直线移动 1.0，对角移动 sqrt(2)）
      double step_cost = (dx[i] != 0 && dy[i] != 0) ? 1.414 : 1.0;

      Node neighbor(nx, ny);
      neighbor.g = current.g + step_cost;
      neighbor.h = getHeuristic(nx, ny, goal_gx, goal_gy);
      neighbor.f = neighbor.g + neighbor.h;
      neighbor.parent_x = current.x;
      neighbor.parent_y = current.y;

      open_set.push(neighbor);
      visited_nodes_.push_back(neighbor);
    }
  }

  ROS_WARN("[A*] plan failed! no feasible path found.");
  return false;
}

} // namespace global_planner_learning
