/**
 * @file astar_planner.h
 * @brief A* 路径规划算法（基于栅格地图）
 * @author learner
 * @version 0.1
 *
 * 继承 PathPlannerBase 基类的 A* 实现。
 * 支持 4/8 连通、允许/禁止未知区域穿越。
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <limits>
#include <nav_msgs/OccupancyGrid.h>
#include <queue>
#include <vector>

#include "global_planner_learning/planner_base.h"

namespace global_planner_learning {

/**
 * @brief A* 路径规划器
 *
 * 在 nav_msgs::OccupancyGrid 地图上执行 A* 搜索。
 * 支持 4 连通和 8 连通邻域搜索。
 */
class AStarPlanner : public PathPlannerBase {
public:
  /**
   * @brief 构造函数
   * @param map 输入的地图（OccupancyGrid 格式）
   * @param allow_unknown 是否允许穿越未知区域
   * @param use_8_connectivity 是否使用 8 连通（默认 true）
   */
  AStarPlanner(const nav_msgs::OccupancyGrid::ConstPtr &map,
               bool allow_unknown = false, bool use_8_connectivity = true);

  // ==================== 基类接口覆盖 ====================

  bool makePlan(const geometry_msgs::PoseStamped &start_world,
                const geometry_msgs::PoseStamped &goal_world,
                std::vector<geometry_msgs::PoseStamped> &plan) override;

  PlannerStatistics getStatistics() const override { return stats_; }

  std::string getName() const override { return "AStar"; }

  const std::vector<Node> &getVisitedNodes() const override {
    return visited_nodes_;
  }

private:
  /**
   * @brief 世界坐标 → 栅格坐标
   */
  bool worldToGrid(double wx, double wy, int &gx, int &gy) const;

  /**
   * @brief 栅格坐标 → 世界坐标
   */
  void gridToWorld(int gx, int gy, double &wx, double &wy) const;

  /**
   * @brief 检查栅格是否可通过
   */
  bool isWalkable(int gx, int gy) const;

  /**
   * @brief 启发函数（欧氏距离）
   */
  double getHeuristic(int gx, int gy, int goal_x, int goal_y) const;

  /**
   * @brief 重建路径（从目标点回溯到起点）
   */
  void reconstructPath(int goal_x, int goal_y,
                       std::vector<geometry_msgs::PoseStamped> &plan);

private:
  const nav_msgs::OccupancyGrid *map_; // 地图数据指针
  bool allow_unknown_;                 // 是否允许通过未知区域
  bool use_8_connectivity_;            // 是否使用 8 连通

  int width_;                  // 地图宽度（像素）
  int height_;                 // 地图高度（像素）
  double resolution_;          // 地图分辨率（米/像素）
  double origin_x_, origin_y_; // 地图原点（世界坐标）

  PlannerStatistics stats_; // 规划统计（每次 makePlan 后更新）

  std::vector<Node> visited_nodes_; // 搜索过程中访问的节点（可视化用）

  // 父节点追踪数组：parent_[y*width+x] = {parent_x, parent_y}
  // 在 A* 搜索过程中记录每个栅格首次从 open_set 弹出时的最优父节点
  std::vector<std::pair<int, int>> parent_;

  /**
   * @brief 计算路径总长度（辅助统计用）
   */
  double
  computePathLength(const std::vector<geometry_msgs::PoseStamped> &plan) const;

  // 邻域偏移量
  static constexpr int DX4_[4] = {1, -1, 0, 0};
  static constexpr int DY4_[4] = {0, 0, 1, -1};
  static constexpr int DX8_[8] = {1, -1, 0, 0, 1, -1, 1, -1};
  static constexpr int DY8_[8] = {0, 0, 1, -1, 1, 1, -1, -1};

  // 阈值常量
  static constexpr int8_t OCCUPIED_THRESH = 50; // > 50 视为障碍物
  static constexpr int8_t UNKNOWN_VALUE = -1;
};

} // namespace global_planner_learning
