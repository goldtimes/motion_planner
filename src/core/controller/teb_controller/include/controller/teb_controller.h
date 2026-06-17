/**
 * *********************************************************
 *
 * @file: teb_controller.h
 * @brief: Contains the TEB (Timed Elastic Band) local planner ROS wrapper class
 * @author: Generated
 * @date: 2024-01-31
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef RMP_CONTROLLER_TEB_CONTROLLER_H_
#define RMP_CONTROLLER_TEB_CONTROLLER_H_

#include <memory>
#include <vector>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <angles/angles.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/buffer.h>

#include <base_local_planner/latched_stop_rotate_controller.h>
#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>

#include <dynamic_reconfigure/server.h>

#include "controller/teb_optimizer.h"
#include "teb_controller/TEBControllerConfig.h"

namespace rmp::controller {

/**
 * @class TEBController
 * @brief ROS Wrapper for the TEB local planner that adheres to the
 * BaseLocalPlanner interface and can be used as a plugin for move_base.
 *
 * The Timed Elastic Band (TEB) approach optimizes a trajectory represented
 * as a sequence of robot poses with associated time intervals using
 * Ceres-solver for sparse nonlinear optimization.
 */
class TEBController : public nav_core::BaseLocalPlanner {
public:
  /**
   * @brief Constructor for TEBController
   */
  TEBController();

  /**
   * @brief Destructor
   */
  ~TEBController() override;

  /**
   * @brief Initialize the planner
   * @param name the name to give this instance
   * @param tf a pointer to a transform listener
   * @param costmap_ros the cost map to use
   */
  void initialize(std::string name, tf2_ros::Buffer *tf,
                  costmap_2d::Costmap2DROS *costmap_ros) override;

  /**
   * @brief Set the plan that the controller is following
   * @param orig_global_plan the plan to pass to the controller
   * @return true if the plan was updated successfully
   */
  bool setPlan(
      const std::vector<geometry_msgs::PoseStamped> &orig_global_plan) override;

  /**
   * @brief Compute velocity commands for the robot
   * @param cmd_vel will be filled with the velocity command
   * @return true if a valid trajectory was found
   */
  bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel) override;

  /**
   * @brief Check if the goal has been reached
   * @return true if achieved
   */
  bool isGoalReached() override;

  /**
   * @brief Check if initialized
   */
  bool isInitialized() const { return initialized_; }

private:
  /**
   * @brief Dynamic reconfigure callback
   */
  void reconfigureCB(teb_controller::TEBControllerConfig &config,
                     uint32_t level);

  /**
   * @brief Publish the local plan for visualization
   */
  void publishLocalPlan(const std::vector<geometry_msgs::PoseStamped> &path);

  /**
   * @brief Publish the global plan for visualization
   */
  void publishGlobalPlan(const std::vector<geometry_msgs::PoseStamped> &path);

  /**
   * @brief Transform a pose from one frame to another
   */
  void transformPose(const std::string &target_frame,
                     const geometry_msgs::PoseStamped &in_pose,
                     geometry_msgs::PoseStamped &out_pose) const;

  /**
   * @brief Prune the global plan to remove passed waypoints
   */
  std::vector<geometry_msgs::PoseStamped>
  prunePlan(const geometry_msgs::PoseStamped &robot_pose) const;

  /**
   * @brief Update the configuration from protobuf
   */
  void updateConfigFromProtobuf();

private:
  bool initialized_{false};
  bool goal_reached_{false};
  bool setup_{false};

  // TF
  tf2_ros::Buffer *tf_{nullptr};

  // Costmap
  costmap_2d::Costmap2DROS *costmap_ros_{nullptr};

  // Local planner utility (for parameter handling)
  base_local_planner::LocalPlannerUtil planner_util_;

  // TEB optimizer
  std::unique_ptr<TEBOptimizer> teb_optimizer_;

  // Latched stop rotate controller for final goal adjustment
  base_local_planner::LatchedStopRotateController
      latched_stop_rotate_controller_;

  // Odometry helper
  base_local_planner::OdometryHelperRos odom_helper_;
  std::string odom_topic_{"odom"};

  // Current robot pose
  geometry_msgs::PoseStamped current_pose_;

  // Global plan
  std::vector<geometry_msgs::PoseStamped> global_plan_;

  // Goal pose
  double goal_x_{0.0}, goal_y_{0.0}, goal_theta_{0.0};

  // Publishers for visualization
  ros::Publisher g_plan_pub_, l_plan_pub_;
  ros::Publisher teb_trajectory_pub_;

  // Dynamic reconfigure server
  dynamic_reconfigure::Server<teb_controller::TEBControllerConfig> *dsrv_{
      nullptr};
  teb_controller::TEBControllerConfig default_config_;
};

} // namespace rmp::controller

#endif // RMP_CONTROLLER_TEB_CONTROLLER_H_
