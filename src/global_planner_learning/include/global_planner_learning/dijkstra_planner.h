/**
 * @file dijkstra_planner.h
 * @brief Dijkstra 路径规划算法（基于栅格地图）
 * @author learner
 * @version 0.1
 *
 * 继承 PathPlannerBase 基类的 Dijkstra 实现。
 * 支持 4/8 连通、允许/禁止未知区域穿越。
 *
 * 与 A* 的区别：Dijkstra 不使用启发函数（h=0），
 * 仅依靠从起点出发的实际代价值 g 进行扩展，
 * 保证找到最短路径，但搜索效率通常低于 A*。
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
 * @brief Dijkstra 路径规划器
 *
 * 在 nav_msgs::OccupancyGrid 地图上执行 Dijkstra 搜索。
 * 与 A* 的核心区别：不使用启发函数（h ≡ 0），
 * 等价于 A* 在 heuristic_weight = 0 时的特例。
 *
 * 算法特性：
 * - 保证找到最短路径（最优性）
 * - 搜索范围呈同心圆状向外扩张，效率低于 A*
 * - 对于任意图结构均能保证完备性
 */
class DijkstraPlanner : public PathPlannerBase {
public:
  /**
   * @brief 构造函数
   * @param map 输入的地图（OccupancyGrid 格式）
   * @param allow_unknown 是否允许穿越未知区域
   * @param use_8_connectivity 是否使用 8 连通（默认 true）
   */
  DijkstraPlanner(const nav_msgs::OccupancyGrid::ConstPtr &map,
                  bool allow_unknown = false, bool use_8_connectivity = true);

  // ==================== 基类接口覆盖 ====================

  bool makePlan(const geometry_msgs::PoseStamped &start_world,
                const geometry_msgs::PoseStamped &goal_world,
                std::vector<geometry_msgs::PoseStamped> &plan) override;

  PlannerStatistics getStatistics() const override { return stats_; }

  std::string getName() const override { return "Dijkstra"; }

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
   * @brief 重建路径（从目标点回溯到起点）
   */
  void reconstructPath(int goal_x, int goal_y,
                       std::vector<geometry_msgs::PoseStamped> &plan);

  /**
   * @brief 计算路径总长度（辅助统计用）
   */
  double
  computePathLength(const std::vector<geometry_msgs::PoseStamped> &plan) const;

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
  std::vector<std::pair<int, int>> parent_;

  // 邻域偏移量
  static constexpr int DX4_[4] = {1, -1, 0, 0};
  static constexpr int DY4_[4] = {0, 0, 1, -1};
  static constexpr int DX8_[8] = {1, -1, 0, 0, 1, -1, 1, -1};
  static constexpr int DY8_[8] = {0, 0, 1, -1, 1, 1, -1, -1};

  // 阈值常量
  static constexpr int8_t OCCUPIED_THRESH = 50;
  static constexpr int8_t UNKNOWN_VALUE = -1;
};

} // namespace global_planner_learning
