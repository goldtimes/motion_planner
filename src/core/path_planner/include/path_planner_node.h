/**
 * @file path_planner_node.h
 * @author 李航
 * @brief 继承ros的全局路径规划节点
 * @version 0.1
 */

#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <memory>

namespace rmp::path_planner {
class PathPlanner;
class PathPlannerNode : public nav_core::BaseGlobalPlanner {
public:
  PathPlannerNode();
  PathPlannerNode(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
  ~PathPlannerNode() = default;
  void initialize(std::string name, costmap_2d::Costmap2DROS *costmapRos);
  void initialize(std::string name);
  bool makePlan(const geometry_msgs::PoseStamped &start,
                const geometry_msgs::PoseStamped &goal,
                std::vector<geometry_msgs::PoseStamped> &plan);
  bool makePlan(const geometry_msgs::PoseStamped &start,
                const geometry_msgs::PoseStamped &goal, double tolerance,
                std::vector<geometry_msgs::PoseStamped> &plan);

private:
    bool initialized_;                        // initialization flag
    costmap_2d::Costmap2DROS* costmap_ros_;   // costmap(ROS wrapper)
    std::string frame_id_;                    // costmap frame ID
    std::shared_ptr<PathPlanner> g_planner_;
    ros::Publisher plan_pub_;                 // path planning publisher
    ros::Publisher expand_pub_;               // nodes explorer publisher
    ros::Publisher points_pub_;               // key-points publisher
    ros::Publisher lines_pub_;                // polygons publisher
    ros::Publisher tree_pub_;                 // random search tree publisher
    ros::Publisher particles_pub_;            // evolutionary particles publisher
    ros::ServiceServer make_plan_srv_;        // planning service
};
} // namespace rmp::path_planner