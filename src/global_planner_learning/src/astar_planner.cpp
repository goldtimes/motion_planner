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

// ==================== 路径长度计算 ====================
double AStarPlanner::computePathLength(
    const std::vector<geometry_msgs::PoseStamped> &plan) const {
  double length = 0.0;
  for (size_t i = 1; i < plan.size(); ++i) {
    double dx = plan[i].pose.position.x - plan[i - 1].pose.position.x;
    double dy = plan[i].pose.position.y - plan[i - 1].pose.position.y;
    length += std::sqrt(dx * dx + dy * dy);
  }
  return length;
}

// ==================== 路径回溯 ====================
void AStarPlanner::reconstructPath(
    int goal_x, int goal_y, std::vector<geometry_msgs::PoseStamped> &plan) {
  ROS_INFO("[A*] reconstruct path from goal: (%d, %d)", goal_x, goal_y);
  std::vector<geometry_msgs::PoseStamped> path_nodes;

  int cx = goal_x, cy = goal_y;
  int iter = 0;
  const int max_iter = width_ * height_; // 安全上限

  // 沿着 parent_ 数组从目标回溯到起点（起点 parent 为 {-1, -1}）
  while (cx >= 0 && cy >= 0 && iter < max_iter) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = map_->header.frame_id;
    pose.header.stamp = ros::Time(0);
    gridToWorld(cx, cy, pose.pose.position.x, pose.pose.position.y);
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    path_nodes.push_back(pose);

    int idx = cy * width_ + cx;
    const auto &p = parent_[idx];
    if (p.first < 0 || p.second < 0)
      break; // 到达起点
    cx = p.first;
    cy = p.second;
    ++iter;
  }

  if (iter >= max_iter) {
    ROS_ERROR("[A*] reconstructPath: exceeded max iterations, possible cycle!");
    plan.clear();
    return;
  }

  // 反转路径（从起点到终点）
  std::reverse(path_nodes.begin(), path_nodes.end());
  plan = path_nodes;
}

// ==================== A* 主算法 ====================
bool AStarPlanner::makePlan(const geometry_msgs::PoseStamped &start_world,
                            const geometry_msgs::PoseStamped &goal_world,
                            std::vector<geometry_msgs::PoseStamped> &plan) {
  // ---- 计时开始 ----
  ros::Time start_time = ros::Time::now();

  plan.clear();
  visited_nodes_.clear();
  stats_ = PlannerStatistics(); // 重置统计
  stats_.planner_name = getName();

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
  // closed set：标记已确定最优路径的节点
  std::vector<bool> closed_set(width_ * height_, false);

  // g_value：记录从起点到每个栅格的最佳代价值，用于跳过更差的路径
  std::vector<double> g_value(width_ * height_,
                              std::numeric_limits<double>::max());

  // parent_：记录每个栅格在最优路径上的父节点
  parent_.assign(width_ * height_, {-1, -1});

  // 优先队列 (open set)，最小堆
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_set;

  // 起点
  Node start_node(start_gx, start_gy);
  start_node.g = 0.0;
  start_node.h = getHeuristic(start_gx, start_gy, goal_gx, goal_gy);
  start_node.f = start_node.g + start_node.h;
  start_node.parent_x = -1; // 起点父节点设为 -1，作为回溯终止标记
  start_node.parent_y = -1;

  open_set.push(start_node);
  visited_nodes_.push_back(start_node);
  g_value[start_gy * width_ + start_gx] = 0.0;

  // 选择邻域偏移
  const int *dx = use_8_connectivity_ ? DX8_ : DX4_;
  const int *dy = use_8_connectivity_ ? DY8_ : DX4_;
  int num_neighbors = use_8_connectivity_ ? 8 : 4;

  // 4. A* 主循环
  while (!open_set.empty()) {
    // 取出 f 值最小的节点
    Node current = open_set.top();
    open_set.pop();

    int idx = current.y * width_ + current.x;

    // 如果已经在 closed set 中，跳过（该节点已有最优路径）
    if (closed_set[idx])
      continue;

    // 标记为已访问，并记录父节点（这是首次弹出，路径最优）
    closed_set[idx] = true;
    parent_[idx] = {current.parent_x, current.parent_y};

    // 到达目标点（精确到栅格即可）
    if (current.x == goal_gx && current.y == goal_gy) {
      reconstructPath(goal_gx, goal_gy, plan);

      // ---- 统计：成功 ----
      ros::Time end_time = ros::Time::now();
      stats_.success = true;
      stats_.planning_time_ms = (end_time - start_time).toSec() * 1000.0;
      stats_.visited_nodes_count = visited_nodes_.size();
      stats_.path_points_count = plan.size();
      stats_.path_length_m = computePathLength(plan);

      stats_.log();
      return true;
    }

    // 遍历邻居
    for (int i = 0; i < num_neighbors; ++i) {
      int nx = current.x + dx[i];
      int ny = current.y + dy[i];

      if (!isWalkable(nx, ny))
        continue;

      int nidx = ny * width_ + nx;
      if (closed_set[nidx])
        continue;

      // 计算移动代价（直线移动 1.0，对角移动 sqrt(2)）
      double step_cost = (dx[i] != 0 && dy[i] != 0) ? 1.414 : 1.0;
      double new_g = current.g + step_cost;

      // 如果已经找到一条更短的路径到该节点，跳过
      if (new_g >= g_value[nidx])
        continue;

      // 更新最优代价值
      g_value[nidx] = new_g;

      Node neighbor(nx, ny);
      neighbor.g = new_g;
      neighbor.h = getHeuristic(nx, ny, goal_gx, goal_gy);
      neighbor.f = neighbor.g + neighbor.h;
      neighbor.parent_x = current.x;
      neighbor.parent_y = current.y;

      open_set.push(neighbor);
      visited_nodes_.push_back(neighbor);
    }
  }

  // ---- 统计：失败 ----
  ros::Time end_time = ros::Time::now();
  stats_.success = false;
  stats_.planning_time_ms = (end_time - start_time).toSec() * 1000.0;
  stats_.visited_nodes_count = visited_nodes_.size();
  stats_.path_points_count = 0;
  stats_.path_length_m = 0.0;

  stats_.log();
  return false;
}

} // namespace global_planner_learning
