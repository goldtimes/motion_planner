/**
 * @file rrt_planner.cpp
 * @brief RRT 路径规划核心算法实现
 * @author learner
 * @version 0.1
 *
 * 算法原理：
 * RRT（Rapidly-exploring Random Tree）通过在连续空间中随机采样，
 * 从起点开始快速扩展一棵树，直到到达目标区域。
 *
 * 核心步骤（每轮迭代）：
 * ① sample()   — 在自由空间中随机采样（带目标偏置）
 * ② nearest()  — 在树中找到最近节点
 * ③ steer()    — 向采样点方向步进
 * ④ collision_free() — 检查路径是否与障碍物碰撞
 * ⑤ 若无碰撞则加入新节点
 *
 * 本实现基于栅格地图进行碰撞检测，
 * 使用 Bresenham 线段算法判断线段是否穿越障碍物。
 */

#include "global_planner_learning/rrt_planner.h"
#include <ctime>
#include <ros/ros.h>

namespace global_planner_learning {

// ==================== 构造函数 ====================
RRTPlanner::RRTPlanner(const nav_msgs::OccupancyGrid::ConstPtr &map,
                       bool allow_unknown, double goal_bias, double step_size,
                       int max_iter)
    : map_(map.get()), allow_unknown_(allow_unknown), goal_bias_(goal_bias),
      step_size_(step_size), max_iter_(max_iter), goal_tolerance_(step_size) {
  // 初始化随机数种子
  std::srand(static_cast<unsigned>(std::time(nullptr)));
  width_ = map->info.width;
  height_ = map->info.height;
  resolution_ = map->info.resolution;
  origin_x_ = map->info.origin.position.x;
  origin_y_ = map->info.origin.position.y;

  // 计算地图边界（世界坐标）
  map_x_min_ = origin_x_;
  map_x_max_ = origin_x_ + width_ * resolution_;
  map_y_min_ = origin_y_;
  map_y_max_ = origin_y_ + height_ * resolution_;
}

// ==================== 坐标转换 ====================
bool RRTPlanner::worldToGrid(double wx, double wy, int &gx, int &gy) const {
  gx = static_cast<int>((wx - origin_x_) / resolution_);
  gy = static_cast<int>((wy - origin_y_) / resolution_);
  return (gx >= 0 && gx < width_ && gy >= 0 && gy < height_);
}

void RRTPlanner::gridToWorld(int gx, int gy, double &wx, double &wy) const {
  wx = origin_x_ + (gx + 0.5) * resolution_;
  wy = origin_y_ + (gy + 0.5) * resolution_;
}

// ==================== 碰撞检测 ====================
bool RRTPlanner::isWalkable(int gx, int gy) const {
  if (gx < 0 || gx >= width_ || gy < 0 || gy >= height_)
    return false;

  int8_t cost = map_->data[gy * width_ + gx];

  if (cost == UNKNOWN_VALUE)
    return allow_unknown_;

  if (cost >= OCCUPIED_THRESH)
    return false;

  return true;
}

bool RRTPlanner::isCollisionFree(double ax, double ay, double bx,
                                 double by) const {
  // 将端点转换为栅格坐标
  int gx0, gy0, gx1, gy1;
  if (!worldToGrid(ax, ay, gx0, gy0))
    return false;
  if (!worldToGrid(bx, by, gx1, gy1))
    return false;

  // 使用 Bresenham 线段算法遍历路径上的所有栅格
  int dx = std::abs(gx1 - gx0);
  int dy = std::abs(gy1 - gy0);
  int sx = (gx0 < gx1) ? 1 : -1;
  int sy = (gy0 < gy1) ? 1 : -1;
  int err = dx - dy;

  int cx = gx0, cy = gy0;

  while (true) {
    // 检查当前栅格是否可通行
    if (!isWalkable(cx, cy))
      return false;

    // 到达终点
    if (cx == gx1 && cy == gy1)
      break;

    int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      cx += sx;
    }
    if (e2 < dx) {
      err += dx;
      cy += sy;
    }
  }

  return true;
}

// ==================== 随机采样 ====================
std::pair<double, double> RRTPlanner::sample() {
  // 目标偏置：以 goal_bias_ 概率直接返回目标点
  double r = static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX);
  if (r < goal_bias_) {
    return {goal_wx_, goal_wy_};
  }

  // 均匀随机采样
  double rx = static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX);
  double ry = static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX);
  double x = map_x_min_ + rx * (map_x_max_ - map_x_min_);
  double y = map_y_min_ + ry * (map_y_max_ - map_y_min_);
  return {x, y};
}

// ==================== 最近邻查找 ====================
int RRTPlanner::nearest(double rx, double ry) const {
  int best_idx = 0;
  double best_dist = std::numeric_limits<double>::max();

  for (size_t i = 0; i < nodes_.size(); ++i) {
    double dx = nodes_[i].x - rx;
    double dy = nodes_[i].y - ry;
    double dist = dx * dx + dy * dy; // 比较平方距离，避免开方

    if (dist < best_dist) {
      best_dist = dist;
      best_idx = static_cast<int>(i);
    }
  }

  return best_idx;
}

// ==================== 步进扩展 ====================
void RRTPlanner::steer(double nx, double ny, double rx, double ry,
                       double &new_x, double &new_y) const {
  double dx = rx - nx;
  double dy = ry - ny;
  double dist = std::sqrt(dx * dx + dy * dy);

  if (dist < step_size_) {
    // 距离小于步长，直接到达采样点
    new_x = rx;
    new_y = ry;
  } else {
    // 按步长步进
    new_x = nx + (dx / dist) * step_size_;
    new_y = ny + (dy / dist) * step_size_;
  }
}

// ==================== 路径回溯 ====================
void RRTPlanner::reconstructPath(
    int goal_idx, std::vector<geometry_msgs::PoseStamped> &plan) {
  ROS_INFO("[RRT] reconstruct path, goal node index: %d (%d nodes total)",
           goal_idx, (int)nodes_.size());

  std::vector<geometry_msgs::PoseStamped> path_nodes;

  int current = goal_idx;
  int iter = 0;
  const int max_iter = max_iter_; // 安全上限

  while (current >= 0 && iter < max_iter) {
    const RRTNode &node = nodes_[current];

    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = map_->header.frame_id;
    pose.header.stamp = ros::Time(0);
    pose.pose.position.x = node.x;
    pose.pose.position.y = node.y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    path_nodes.push_back(pose);

    current = node.parent_idx;
    ++iter;
  }

  if (iter >= max_iter) {
    ROS_ERROR("[RRT] reconstructPath: exceeded max iterations!");
    plan.clear();
    return;
  }

  // 反转路径（从起点到终点）
  std::reverse(path_nodes.begin(), path_nodes.end());
  plan = path_nodes;
}

// ==================== 路径长度计算 ====================
double RRTPlanner::computePathLength(
    const std::vector<geometry_msgs::PoseStamped> &plan) const {
  double length = 0.0;
  for (size_t i = 1; i < plan.size(); ++i) {
    double dx = plan[i].pose.position.x - plan[i - 1].pose.position.x;
    double dy = plan[i].pose.position.y - plan[i - 1].pose.position.y;
    length += std::sqrt(dx * dx + dy * dy);
  }
  return length;
}

// ==================== 同步 visited_nodes ====================
void RRTPlanner::syncVisitedNodes() {
  visited_nodes_.clear();
  visited_nodes_.reserve(nodes_.size());

  for (const auto &rrt_node : nodes_) {
    Node n;
    n.x = rrt_node.gx;
    n.y = rrt_node.gy;
    n.g = rrt_node.cost;
    n.h = 0.0;
    n.f = n.g;
    n.parent_x =
        (rrt_node.parent_idx >= 0) ? nodes_[rrt_node.parent_idx].gx : -1;
    n.parent_y =
        (rrt_node.parent_idx >= 0) ? nodes_[rrt_node.parent_idx].gy : -1;
    visited_nodes_.push_back(n);
  }
}

// ==================== RRT 主算法 ====================
bool RRTPlanner::makePlan(const geometry_msgs::PoseStamped &start_world,
                          const geometry_msgs::PoseStamped &goal_world,
                          std::vector<geometry_msgs::PoseStamped> &plan) {
  // ---- 计时开始 ----
  ros::Time start_time = ros::Time::now();

  plan.clear();
  nodes_.clear();
  visited_nodes_.clear();
  stats_ = PlannerStatistics();
  stats_.planner_name = getName();

  // 1. 缓存起点/终点的世界坐标
  start_wx_ = start_world.pose.position.x;
  start_wy_ = start_world.pose.position.y;
  goal_wx_ = goal_world.pose.position.x;
  goal_wy_ = goal_world.pose.position.y;

  // 2. 世界坐标 → 栅格坐标
  int start_gx, start_gy;
  if (!worldToGrid(start_wx_, start_wy_, start_gx, start_gy)) {
    ROS_WARN("[RRT] start pose out of map range!");
    return false;
  }
  if (!worldToGrid(goal_wx_, goal_wy_, goal_gx_, goal_gy_)) {
    ROS_WARN("[RRT] goal pose out of map range!");
    return false;
  }

  // 3. 检查起点和终点是否可通行
  if (!isWalkable(start_gx, start_gy)) {
    ROS_WARN("[RRT] start pose on obstacle!");
    return false;
  }
  if (!isWalkable(goal_gx_, goal_gy_)) {
    ROS_WARN("[RRT] goal pose on obstacle!");
    return false;
  }

  // 4. 初始化 RRT 树：加入起点
  nodes_.emplace_back(start_wx_, start_wy_, start_gx, start_gy);
  nodes_.back().parent_idx = -1;
  nodes_.back().cost = 0.0;

  ROS_INFO("[RRT] start RRT exploration, step=%.2fm, bias=%.2f, max_iter=%d",
           step_size_, goal_bias_, max_iter_);

  // 5. RRT 主循环
  int goal_node_idx = -1;

  for (int iter = 0; iter < max_iter_; ++iter) {
    // ① 随机采样
    std::pair<double, double> sampled = sample();
    double rx = sampled.first;
    double ry = sampled.second;

    // ② 最近邻查找
    int near_idx = nearest(rx, ry);

    // ③ 步进扩展
    double new_x, new_y;
    steer(nodes_[near_idx].x, nodes_[near_idx].y, rx, ry, new_x, new_y);

    // ④ 碰撞检测
    if (!isCollisionFree(nodes_[near_idx].x, nodes_[near_idx].y, new_x,
                         new_y)) {
      continue;
    }

    // ⑤ 将新节点加入树
    int new_gx, new_gy;
    worldToGrid(new_x, new_y, new_gx, new_gy);

    double dx = new_x - nodes_[near_idx].x;
    double dy = new_y - nodes_[near_idx].y;
    double step_cost = std::sqrt(dx * dx + dy * dy);

    nodes_.emplace_back(new_x, new_y, new_gx, new_gy);
    nodes_.back().parent_idx = near_idx;
    nodes_.back().cost = nodes_[near_idx].cost + step_cost;

    // ⑥ 检查是否到达目标区域
    double dist_to_goal = std::sqrt((new_x - goal_wx_) * (new_x - goal_wx_) +
                                    (new_y - goal_wy_) * (new_y - goal_wy_));

    if (dist_to_goal < goal_tolerance_) {
      // 将目标点本身也加入树（连接到新节点）
      dx = goal_wx_ - new_x;
      dy = goal_wy_ - new_y;
      double final_cost = std::sqrt(dx * dx + dy * dy);

      // 检查到目标点的最后一段是否无碰撞
      if (isCollisionFree(new_x, new_y, goal_wx_, goal_wy_)) {
        nodes_.emplace_back(goal_wx_, goal_wy_, goal_gx_, goal_gy_);
        nodes_.back().parent_idx = static_cast<int>(nodes_.size()) - 2;
        nodes_.back().cost = nodes_[nodes_.size() - 2].cost + final_cost;
        goal_node_idx = static_cast<int>(nodes_.size()) - 1;

        ROS_INFO("[RRT] reached goal after %d iterations!", iter + 1);
        break;
      }
    }

    // 定期输出进度（每 500 次迭代）
    if ((iter + 1) % 500 == 0) {
      ROS_DEBUG("[RRT] iteration %d, tree size: %zu", iter + 1, nodes_.size());
    }
  }

  // ---- 统计 & 输出 ----
  ros::Time end_time = ros::Time::now();
  stats_.planning_time_ms = (end_time - start_time).toSec() * 1000.0;
  stats_.visited_nodes_count = nodes_.size();

  // 同步 visited_nodes（用于可视化）
  syncVisitedNodes();

  if (goal_node_idx >= 0) {
    // 路径回溯
    reconstructPath(goal_node_idx, plan);

    stats_.success = true;
    stats_.path_points_count = plan.size();
    stats_.path_length_m = computePathLength(plan);
    stats_.log();
    return true;
  } else {
    // 取离目标最近的节点作为次优解
    int best_idx = nearest(goal_wx_, goal_wy_);
    double best_dist = std::sqrt(
        (nodes_[best_idx].x - goal_wx_) * (nodes_[best_idx].x - goal_wx_) +
        (nodes_[best_idx].y - goal_wy_) * (nodes_[best_idx].y - goal_wy_));

    ROS_WARN("[RRT] failed to reach goal after %d iterations "
             "(nearest dist: %.2f m, tree size: %zu)",
             max_iter_, best_dist, nodes_.size());

    stats_.success = false;
    stats_.path_points_count = 0;
    stats_.path_length_m = 0.0;
    stats_.log();
    return false;
  }
}

} // namespace global_planner_learning
