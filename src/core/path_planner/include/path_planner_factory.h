#ifndef RMP_PATH_PLANNER_UTILS_PATH_PLANNER_FACTORY_H_
#define RMP_PATH_PLANNER_UTILS_PATH_PLANNER_FACTORY_H_

/**
 * @brief PathPlanner 工厂类
 * @author 李航
 * @version 0.1
 *
 * 用于创建和配置路径规划器实例。
 */
#include <ros/ros.h>

#include "graph_planner/astar_planner.h"
#include "path_planner.h"
#include "sample_planner/rrt_planner.h"

namespace rmp::path_planner {
enum PLANNER_TYPE {
  GRAPH_PLANNER = 0,  // 基于图的路径规划器
  SAMPLE_PLANNER = 1, // 基于采样的路径规划器
  EVOLUTION_PLANNER = 2,
};

class PathPlannerFactory {
public:
  struct PlannerProps {
    PLANNER_TYPE planner_type;
    std::shared_ptr<PathPlanner> planner_ptr; // global path planner
  };

public:
  /**
   * @brief Create and configure planner
   * @param nh ROS node handler
   * @param costmap_ros costmap ROS wrapper
   * @param planner_props planner property
   * @return bool true if create successful, else false
   */
  static bool createPlanner(ros::NodeHandle &nh,
                            costmap_2d::Costmap2DROS *costmap_ros,
                            PlannerProps &planner_props);
};
} // namespace rmp::path_planner

#endif