/**
 * @file rrt_planner.h
 * @brief RRT 路径规划算法（基于栅格地图）
 * @author learner
 * @version 0.1
 *
 * 继承 PathPlannerBase 基类的基本 RRT 实现。
 *
 * 算法特点：
 * - 基于随机采样的搜索，适用于高维或复杂环境
 * - 概率完备（采样点足够多时保证找到可行路径）
 * - 不保证最优性，但可通过后处理平滑路径
 * - 支持目标偏置采样以加速收敛
 *
 * 与 A* / Dijkstra 的区别：
 * - 在连续空间中进行搜索，非离散栅格搜索
 * - 通过随机采样 + 碰撞检测扩展树结构
 * - 不需要启发函数，也不需要 4/8 连通邻域
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <limits>
#include <nav_msgs/OccupancyGrid.h>
#include <cstdlib>
#include <vector>

#include "global_planner_learning/planner_base.h"

namespace global_planner_learning {

/**
 * @brief RRT 内部树节点（连续空间）
 *
 * 与基类 Node 不同，RRT 在连续空间中生长，
 * 因此内部使用双精度坐标存储世界坐标，
 * 仅在碰撞检测和可视化时转换为栅格坐标。
 */
struct RRTNode {
  double x, y;        // 世界坐标（米）
  int gx, gy;         // 对应的栅格坐标
  int parent_idx;     // 父节点在 nodes_ 向量中的索引（-1 表示根节点）
  double cost;        // 从起点到该节点的路径代价

  RRTNode() : x(0), y(0), gx(0), gy(0), parent_idx(-1), cost(0.0) {}
  RRTNode(double wx, double wy, int gxi, int gyi)
      : x(wx), y(wy), gx(gxi), gy(gyi), parent_idx(-1), cost(0.0) {}
};

/**
 * @brief RRT 路径规划器
 *
 * 在 nav_msgs::OccupancyGrid 地图上执行 RRT 搜索。
 * 使用目标偏置采样加速收敛，基于 Bresenham 线段算法进行碰撞检测。
 */
class RRTPlanner : public PathPlannerBase {
public:
  /**
   * @brief 构造函数
   * @param map 输入的地图（OccupancyGrid 格式）
   * @param allow_unknown 是否允许穿越未知区域
   * @param goal_bias 目标偏置概率 [0,1]，默认 0.1（10% 概率直接采样目标）
   * @param step_size RRT 步长（米），默认 0.5
   * @param max_iter 最大迭代次数，默认 5000
   */
  RRTPlanner(const nav_msgs::OccupancyGrid::ConstPtr &map,
             bool allow_unknown = false, double goal_bias = 0.1,
             double step_size = 0.5, int max_iter = 5000);

  // ==================== 基类接口覆盖 ====================

  bool makePlan(const geometry_msgs::PoseStamped &start_world,
                const geometry_msgs::PoseStamped &goal_world,
                std::vector<geometry_msgs::PoseStamped> &plan) override;

  PlannerStatistics getStatistics() const override { return stats_; }

  std::string getName() const override { return "RRT"; }

  const std::vector<Node> &getVisitedNodes() const override {
    return visited_nodes_;
  }

  /**
   * @brief 获取内部树节点（用于调试/可视化）
   */
  const std::vector<RRTNode> &getTreeNodes() const { return nodes_; }

private:
  // ==================== 坐标转换 ====================

  /**
   * @brief 世界坐标 → 栅格坐标
   */
  bool worldToGrid(double wx, double wy, int &gx, int &gy) const;

  /**
   * @brief 栅格坐标 → 世界坐标
   */
  void gridToWorld(int gx, int gy, double &wx, double &wy) const;

  // ==================== 碰撞检测 ====================

  /**
   * @brief 检查单个栅格是否可通过
   */
  bool isWalkable(int gx, int gy) const;

  /**
   * @brief 检查从点 a 到点 b 的线段是否与障碍物碰撞
   * @param ax, ay 起点世界坐标
   * @param bx, by 终点世界坐标
   * @return true 无碰撞, false 碰撞
   *
   * 使用 Bresenham 线段算法遍历路径上的所有栅格，
   * 检查每个栅格是否被占用。
   */
  bool isCollisionFree(double ax, double ay, double bx, double by) const;

  // ==================== RRT 核心步骤 ====================

  /**
   * @brief 在自由空间中随机采样一个点
   * @return 采样点的世界坐标 (x, y)
   *
   * 以 goal_bias 概率直接返回目标点（偏置采样），
   * 否则在地图范围内均匀随机采样。
   */
  std::pair<double, double> sample();

  /**
   * @brief 在树中找到离采样点最近的节点
   * @param rx, ry 采样点世界坐标
   * @return 最近节点在 nodes_ 中的索引
   */
  int nearest(double rx, double ry) const;

  /**
   * @brief 从最近节点向采样点方向步进
   * @param nx, ny 最近节点世界坐标
   * @param rx, ry 采样点世界坐标
   * @param new_x, new_y 输出新节点的世界坐标
   *
   * 从 (nx, ny) 向 (rx, ry) 方向移动 step_size_ 距离。
   * 如果距离小于 step_size_，则直接到达 (rx, ry)。
   */
  void steer(double nx, double ny, double rx, double ry, double &new_x,
             double &new_y) const;

  /**
   * @brief 从目标节点回溯构建路径
   * @param goal_idx 目标节点（离终点最近的节点）索引
   * @param plan 输出路径
   */
  void reconstructPath(int goal_idx,
                       std::vector<geometry_msgs::PoseStamped> &plan);

  /**
   * @brief 计算路径总长度
   */
  double
  computePathLength(const std::vector<geometry_msgs::PoseStamped> &plan) const;

  /**
   * @brief 更新 visited_nodes_（将内部 RRTNode 转为基类 Node）
   */
  void syncVisitedNodes();

private:
  const nav_msgs::OccupancyGrid *map_; // 地图数据指针
  bool allow_unknown_;                 // 是否允许通过未知区域

  int width_;                  // 地图宽度（像素）
  int height_;                 // 地图高度（像素）
  double resolution_;          // 地图分辨率（米/像素）
  double origin_x_, origin_y_; // 地图原点（世界坐标）

  // RRT 参数
  double goal_bias_;   // 目标偏置概率 [0, 1]
  double step_size_;   // 扩展步长（米）
  int max_iter_;       // 最大迭代次数
  double goal_tolerance_; // 到达目标点的判定阈值（米）

  PlannerStatistics stats_; // 规划统计

  // RRT 树结构
  std::vector<RRTNode> nodes_;
  std::vector<Node> visited_nodes_; // 转换后的节点（用于基类接口）

  // 起点和终点的世界坐标（缓存，供 sample 等方法使用）
  double start_wx_, start_wy_;
  double goal_wx_, goal_wy_;
  int goal_gx_, goal_gy_;

  // 地图边界（世界坐标）
  double map_x_min_, map_x_max_;
  double map_y_min_, map_y_max_;

  // 阈值常量
  static constexpr int8_t OCCUPIED_THRESH = 50;
  static constexpr int8_t UNKNOWN_VALUE = -1;
};

} // namespace global_planner_learning
