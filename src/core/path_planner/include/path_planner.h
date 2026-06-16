#pragma once

/**
 * @brief PathPlanner 基类
 * @author 李航
 * @version 0.1
 *
 * 定义了路径规划器的基本接口，包括规划路径和初始化。
 */
#include "collision_checker.h"
#include "common/geometry/point.h"
#include "node.h"
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <ros/ros.h>
#include <unordered_map>

// 父类，定义了路径规划器的基本接口

namespace rmp::path_planner {
class PathPlanner {
public:
  PathPlanner(costmap_2d::Costmap2DROS *costmap);

  virtual ~PathPlanner() = default;
  /**
   * @brief Pure virtual function that is overloadde by planner implementations
   * @param start          start node
   * @param goal           goal node
   * @param path           The resulting path in (x, y, theta)
   * @param expand         containing the node been search during the process
   * @return true if path found, else false
   */
  virtual bool plan(const common::geometry::Point3d &start,
                    const common::geometry::Point3d &end,
                    common::geometry::Points3d *path,
                    common::geometry::Points3d *expand) = 0;
  /**
   * @brief get the costmap
   * @return costmap costmap2d pointer
   */
  costmap_2d::Costmap2D *getCostMap() const;

  /**
   * @brief get the size of costmap
   * @return map_size the size of costmap
   */
  int getMapSize() const;

  /**
   * @brief Transform from grid map(x, y) to grid index(i)
   * @param x grid map x
   * @param y grid map y
   * @return index
   */
  int grid2Index(int x, int y);

  /**
   * @brief Transform from grid index(i) to grid map(x, y)
   * @param i grid index i
   * @param x grid map x
   * @param y grid map y
   */
  void index2Grid(int i, int &x, int &y);

  /**
   * @brief Tranform from world map(x, y) to costmap(x, y)
   * @param mx costmap x
   * @param my costmap y
   * @param wx world map x
   * @param wy world map y
   * @return true if successfull, else false
   */
  bool world2Map(double wx, double wy, double &mx, double &my);

  /**
   * @brief Tranform from costmap(x, y) to world map(x, y)
   * @param mx costmap x
   * @param my costmap y
   * @param wx world map x
   * @param wy world map y
   */
  void map2World(double mx, double my, double &wx, double &wy);

  void outlineMap();

  /**
   * @brief Check the validity of (wx, wy)
   * @param wx world map x
   * @param wy world map y
   * @param mx costmap x
   * @param my costmap y
   * @return flag true if the position is valid
   */
  bool validityCheck(double wx, double wy, double &mx, double &my);

  /**
   * @brief Resample the given path based on a specified sampling ratio
   * @param path           the original path to be resampled
   * @param path_resample  the resulting resampled path
   * @param sample_ratio   the ratio used to determine the sampling intervals
   * @return true if the resampling is successful, false otherwise
   */
  static bool resample(const common::geometry::Points3d &path,
                       common::geometry::Points3d *path_resample,
                       double sample_ratio);

  template <typename Node>
  std::vector<Node>
  _convertClosedListToPath(std::unordered_map<int, Node> &closed_list,
                           const Node &start, const Node &goal) {
    std::vector<Node> path;
    auto current = closed_list.find(goal.id());
    while (current->second != start) {
      path.emplace_back(current->second.x(), current->second.y());
      auto it = closed_list.find(current->second.pid());
      if (it != closed_list.end())
        current = it;
      else
        return {};
    }
    path.push_back(start);
    return path;
  }

protected:
  costmap_2d::Costmap2DROS *costmap_ros_; // costmap ROS wrapper
  costmap_2d::Costmap2D *costmap_;        // costmap buffer

  int nx_, ny_, map_size_; // pixel number in costmap
  //   pb::path_planner::PathPlanner config_;
  std::shared_ptr<rmp::common::geometry::CollisionChecker>
      collision_checker_; // gridmap
                          // collision
                          // checker
};
} // namespace rmp::path_planner
