/**
 * @file planner_base.h
 * @brief 全局路径规划算法抽象基类
 * @author learner
 * @version 0.1
 *
 * 定义所有全局路径规划器必须实现的接口，
 * 以及统一的规划耗时统计数据结构。
 */

#pragma once

#include <algorithm>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <string>
#include <vector>

namespace global_planner_learning {

/**
 * @brief 栅格搜索节点（所有栅格规划器通用）
 */
struct Node {
  int x, y;               // 栅格坐标 (列, 行)
  double g;               // 从起点到当前点的实际代价值
  double h;               // 当前点到目标点的启发式估计值
  double f;               // f = g + h
  int parent_x, parent_y; // 父节点坐标

  Node() : x(0), y(0), g(0), h(0), f(0), parent_x(-1), parent_y(-1) {}
  Node(int x_, int y_)
      : x(x_), y(y_), g(0), h(0), f(0), parent_x(-1), parent_y(-1) {}

  bool operator>(const Node &other) const { return f > other.f; }
};

/**
 * @brief 规划统计算据结构
 *
 * 每次 makePlan 调用后，规划器内部填充此结构，
 * 供外部节点输出到日志、终端或可视化。
 */
struct PlannerStatistics {
  std::string planner_name = "unknown"; // 规划器名称
  bool success = false;                 // 是否规划成功

  double planning_time_ms = 0.0;  // 规划耗时（毫秒）
  size_t visited_nodes_count = 0; // 搜索过程中访问的节点数
  size_t path_points_count = 0;   // 生成的路径点数
  double path_length_m = 0.0;     // 路径总长度（米）

  /** @brief 将统计信息格式化为可读字符串 */
  std::string toString() const {
    std::string s;
    s += "[" + planner_name + "] ";
    s += success ? "SUCCESS" : "FAILED";
    s += "  |  time: " + std::to_string(planning_time_ms) + " ms";
    s += "  |  visited: " + std::to_string(visited_nodes_count) + " nodes";
    s += "  |  path: " + std::to_string(path_points_count) + " pts";
    s += "  |  length: " + std::to_string(path_length_m) + " m";
    return s;
  }

  /** @brief 通过 ROS 日志输出 */
  void log() const {
    if (success) {
      ROS_INFO("%s", toString().c_str());
    } else {
      ROS_WARN("%s", toString().c_str());
    }
  }
};

/**
 * @brief 全局路径规划器抽象基类
 *
 * 所有路径规划算法（A*、Dijkstra、RRT 等）必须继承此类。
 * 使用时通过基类指针多态调用，以便在运行时自由切换算法。
 *
 * 用法示例：
 * @code
 *   PathPlannerBase* planner = new AStarPlanner(map, ...);
 *   planner->makePlan(start, goal, plan);
 *   ROS_INFO("%s", planner->getStatistics().toString().c_str());
 * @endcode
 */
class PathPlannerBase {
public:
  virtual ~PathPlannerBase() = default;

  /**
   * @brief 执行路径规划（核心接口）
   * @param start  起点（世界坐标）
   * @param goal   终点（世界坐标）
   * @param plan   输出的路径点列表
   * @return true 规划成功, false 规划失败
   */
  virtual bool makePlan(const geometry_msgs::PoseStamped &start,
                        const geometry_msgs::PoseStamped &goal,
                        std::vector<geometry_msgs::PoseStamped> &plan) = 0;

  /**
   * @brief 获取规划统计信息（makePlan 后调用）
   */
  virtual PlannerStatistics getStatistics() const = 0;

  /**
   * @brief 获取规划器名称
   */
  virtual std::string getName() const = 0;

  /**
   * @brief 获取搜索过程中访问过的节点（用于可视化）
   */
  virtual const std::vector<Node> &getVisitedNodes() const = 0;
};

} // namespace global_planner_learning
