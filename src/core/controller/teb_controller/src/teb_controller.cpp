/**
 * *********************************************************
 *
 * @file: teb_controller.cpp
 * @brief: Implements the TEB local planner ROS wrapper
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
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "common/util/log.h"
#include "controller/teb_controller.h"

// Register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rmp::controller::TEBController,
                       nav_core::BaseLocalPlanner)

namespace rmp::controller {

TEBController::TEBController()
    : initialized_(false), goal_reached_(false), setup_(false), tf_(nullptr),
      costmap_ros_(nullptr), odom_helper_("odom") {}

TEBController::~TEBController() {
  if (dsrv_) {
    delete dsrv_;
  }
}

void TEBController::initialize(std::string name, tf2_ros::Buffer *tf,
                               costmap_2d::Costmap2DROS *costmap_ros) {
  if (isInitialized()) {
    ROS_WARN("TEB Controller has already been initialized, doing nothing.");
    return;
  }

  ros::NodeHandle private_nh("~/" + name);

  // Store pointers
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  // Publishers for visualization
  g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
  l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
  teb_trajectory_pub_ =
      private_nh.advertise<nav_msgs::Path>("teb_trajectory", 1);

  // Get current pose
  costmap_ros_->getRobotPose(current_pose_);

  // Initialize the local planner utility
  costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();
  planner_util_.initialize(tf, costmap, costmap_ros_->getGlobalFrameID());

  // Create and initialize the TEB optimizer
  teb_optimizer_ = std::make_unique<TEBOptimizer>();

  // Build config from protobuf / parameter server
  TEBConfig teb_config;
  updateConfigFromProtobuf();

  // Read parameters from the parameter server (overrides protobuf defaults)
  private_nh.param("teb_horizon", teb_config.teb_horizon, 40);
  private_nh.param("teb_dt_resolution", teb_config.teb_dt_resolution, 0.15);
  private_nh.param("teb_iterations", teb_config.teb_iterations, 80);

  private_nh.param("weight_path", teb_config.weight_path, 2.0);
  private_nh.param("weight_obstacle", teb_config.weight_obstacle, 5.0);
  private_nh.param("weight_velocity", teb_config.weight_velocity, 1.0);
  private_nh.param("weight_acceleration", teb_config.weight_acceleration, 0.5);
  private_nh.param("weight_time", teb_config.weight_time, 0.5);
  private_nh.param("weight_smoothness", teb_config.weight_smoothness, 2.0);
  private_nh.param("weight_kinematics", teb_config.weight_kinematics, 50.0);

  private_nh.param("max_linear_vel", teb_config.max_linear_vel, 0.5);
  private_nh.param("min_linear_vel", teb_config.min_linear_vel, -0.2);
  private_nh.param("max_angular_vel", teb_config.max_angular_vel, 1.5);
  private_nh.param("min_angular_vel", teb_config.min_angular_vel, -1.5);

  private_nh.param("max_linear_acc", teb_config.max_linear_acc, 1.0);
  private_nh.param("max_angular_acc", teb_config.max_angular_acc, 6.0);

  private_nh.param("obstacle_min_dist", teb_config.obstacle_min_dist, 0.2);
  private_nh.param("inflate_obstacle_radius",
                   teb_config.inflate_obstacle_radius, 0.5);

  private_nh.param("convergence_delta", teb_config.convergence_delta, 1e-3);
  private_nh.param("dt_hysteresis", teb_config.dt_hysteresis, 0.1);

  // Initialize optimizer
  teb_optimizer_->initialize(teb_config, costmap);

  // Odometry topic
  if (private_nh.getParam("odom_topic", odom_topic_)) {
    odom_helper_.setOdomTopic(odom_topic_);
  }

  initialized_ = true;

  R_INFO << "TEB Controller initialized!";

  // Set up dynamic reconfigure
  dsrv_ = new dynamic_reconfigure::Server<teb_controller::TEBControllerConfig>(
      private_nh);
  dynamic_reconfigure::Server<teb_controller::TEBControllerConfig>::CallbackType
      cb = boost::bind(&TEBController::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

void TEBController::updateConfigFromProtobuf() {
  // TODO: Load configuration from protobuf system_config if needed
  // For now, defaults are set in the parameter reading code above
}

void TEBController::reconfigureCB(teb_controller::TEBControllerConfig &config,
                                  uint32_t level) {
  if (setup_ && config.restore_defaults) {
    config = default_config_;
    config.restore_defaults = false;
  }
  if (!setup_) {
    default_config_ = config;
    setup_ = true;
  }

  // Update TEB configuration from dynamic reconfigure
  if (teb_optimizer_) {
    TEBConfig teb_config;
    teb_config.teb_horizon = config.teb_horizon;
    teb_config.teb_dt_resolution = config.teb_dt_resolution;
    teb_config.teb_iterations = config.teb_iterations;
    teb_config.weight_path = config.weight_path;
    teb_config.weight_obstacle = config.weight_obstacle;
    teb_config.weight_velocity = config.weight_velocity;
    teb_config.weight_acceleration = config.weight_acceleration;
    teb_config.weight_time = config.weight_time;
    teb_config.weight_smoothness = config.weight_smoothness;
    teb_config.weight_kinematics = config.weight_kinematics;
    teb_config.max_linear_vel = config.max_linear_vel;
    teb_config.min_linear_vel = config.min_linear_vel;
    teb_config.max_angular_vel = config.max_angular_vel;
    teb_config.min_angular_vel = config.min_angular_vel;
    teb_config.max_linear_acc = config.max_linear_acc;
    teb_config.max_angular_acc = config.max_angular_acc;
    teb_config.obstacle_min_dist = config.obstacle_min_dist;
    teb_config.inflate_obstacle_radius = config.inflate_obstacle_radius;
    teb_config.convergence_delta = config.convergence_delta;
    teb_config.dt_hysteresis = config.dt_hysteresis;

    costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();
    teb_optimizer_->initialize(teb_config, costmap);
  }
}

bool TEBController::setPlan(
    const std::vector<geometry_msgs::PoseStamped> &orig_global_plan) {
  if (!isInitialized()) {
    ROS_ERROR("TEB Controller has not been initialized, please call "
              "initialize() first");
    return false;
  }

  // Reset latching on goal tolerances
  latched_stop_rotate_controller_.resetLatching();

  // Store the global plan
  global_plan_ = orig_global_plan;

  // Also set plan on planner_util_ so LatchedStopRotateController
  // and other base_local_planner utilities can find the goal pose.
  // If the plan's frame_id is empty, fill it with the global frame.
  std::vector<geometry_msgs::PoseStamped> plan_for_util = orig_global_plan;
  if (!plan_for_util.empty() && plan_for_util.front().header.frame_id.empty()) {
    std::string global_frame = costmap_ros_->getGlobalFrameID();
    for (auto &pose : plan_for_util) {
      pose.header.frame_id = global_frame;
    }
    ROS_DEBUG("TEB: Filled empty frame_ids in plan with '%s'",
              global_frame.c_str());
  }
  planner_util_.setPlan(plan_for_util);

  // Update goal pose
  if (!global_plan_.empty()) {
    const auto &goal = global_plan_.back();
    goal_x_ = goal.pose.position.x;
    goal_y_ = goal.pose.position.y;
    goal_theta_ = tf2::getYaw(goal.pose.orientation);
    teb_optimizer_->setGoal(goal_x_, goal_y_, goal_theta_);
  }

  // Set reference plan in optimizer
  teb_optimizer_->setReferencePlan(global_plan_);

  goal_reached_ = false;

  R_INFO << "TEB: Got new plan with " << orig_global_plan.size()
         << " waypoints";
  return true;
}

bool TEBController::computeVelocityCommands(geometry_msgs::Twist &cmd_vel) {
  if (!isInitialized()) {
    ROS_ERROR("TEB Controller has not been initialized");
    return false;
  }

  // Get current robot pose
  if (!costmap_ros_->getRobotPose(current_pose_)) {
    ROS_ERROR("TEB: Could not get robot pose");
    return false;
  }

  // Get robot velocity from odometry
  geometry_msgs::PoseStamped robot_vel;
  odom_helper_.getRobotVel(robot_vel);

  // Current robot state
  double robot_x = current_pose_.pose.position.x;
  double robot_y = current_pose_.pose.position.y;
  double robot_theta = tf2::getYaw(current_pose_.pose.orientation);
  Eigen::Vector3d current_state(robot_x, robot_y, robot_theta);

  // Check if goal is reached (distance check)
  // Only call the latched controller if we have a valid plan with frame_ids.
  bool have_valid_plan =
      !global_plan_.empty() && !global_plan_.back().header.frame_id.empty();
  if (have_valid_plan && latched_stop_rotate_controller_.isGoalReached(
                             &planner_util_, odom_helper_, current_pose_)) {
    goal_reached_ = true;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    return true;
  }

  // Check if we should rotate to goal first (final approach)
  // When close to the goal, stop forward motion and rotate to match the
  // goal orientation. This prevents the TEB look-ahead from commanding
  // the robot to drive past the goal while turning.
  double goal_dist = std::hypot(goal_x_ - current_pose_.pose.position.x,
                                goal_y_ - current_pose_.pose.position.y);
  if (goal_dist < 0.5) {
    double theta = tf2::getYaw(current_pose_.pose.orientation);
    double dir_to_goal = std::atan2(goal_y_ - current_pose_.pose.position.y,
                                    goal_x_ - current_pose_.pose.position.x);
    double angle_diff = angles::shortest_angular_distance(theta, goal_theta_);
    double dir_diff = angles::shortest_angular_distance(theta, dir_to_goal);

    // If both position AND orientation are within tolerance → goal reached
    if (goal_dist < 0.15 && std::abs(angle_diff) < 0.15) {
      goal_reached_ = true;
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
      ROS_INFO("TEB: GOAL Reached! (final approach)");
      return true;
    }

    // Otherwise rotate in place toward the goal
    if (std::abs(angle_diff) > 0.1 || std::abs(dir_diff) > 0.3) {
      double target_angle =
          (std::abs(angle_diff) > std::abs(dir_diff)) ? angle_diff : dir_diff;
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = std::max(-0.8, std::min(target_angle, 0.8));
      ROS_DEBUG("TEB: final approach: goal_dist=%.3f, turn=%.3f", goal_dist,
                target_angle);
      return true;
    }
  }

  // Prune the global plan to remove passed waypoints
  std::vector<geometry_msgs::PoseStamped> prune_plan = prunePlan(current_pose_);

  if (prune_plan.empty()) {
    // If the plan is empty but we're close to the goal, consider it reached.
    double goal_dist = std::hypot(goal_x_ - current_pose_.pose.position.x,
                                  goal_y_ - current_pose_.pose.position.y);
    double angle_diff = angles::shortest_angular_distance(
        current_pose_.pose.position.z, goal_theta_);

    if (goal_dist < 0.3 && std::abs(angle_diff) < 0.3) {
      goal_reached_ = true;
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
      ROS_INFO("TEB: Goal reached (near goal, plan exhausted)");
      return true;
    }

    ROS_WARN("TEB: Pruned plan is empty, stopping");
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    return true;
  }

  // Update reference plan in optimizer
  teb_optimizer_->setReferencePlan(prune_plan);

  // Initialize TEB trajectory from current state and reference plan
  // The initial trajectory is already obstacle-aware (shifts poses away from
  // obstacles before the first optimization).
  teb_optimizer_->initTrajectory(current_state, prune_plan);

  // Run multi-pass optimization: optimize → update obstacle costs → optimize
  // again This allows the obstacle cost to "move" with the trajectory, giving
  // Ceres meaningful gradient information for obstacle avoidance.
  bool optimization_success = false;
  for (int pass = 0; pass < 3; ++pass) {
    optimization_success = teb_optimizer_->optimize();
    if (!optimization_success) {
      break; // no point continuing if optimization fails
    }
    if (pass < 2) {
      // Refresh obstacle costs based on new pose positions, then re-optimize
      teb_optimizer_->updateObstacleCosts();
    }
  }

  if (!optimization_success) {
    // Fallback: try once more from a fresh initialization
    R_WARN << "TEB: Optimization failed, trying fallback...";
    teb_optimizer_->initTrajectory(current_state, prune_plan);
    optimization_success = teb_optimizer_->optimize();

    if (!optimization_success) {
      R_WARN << "TEB: Fallback optimization also failed, stopping robot";
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
      return false;
    }
  }

  // Extract velocity command from optimized trajectory
  double v, w;
  if (!teb_optimizer_->getVelocityCommand(v, w)) {
    ROS_WARN("TEB: Could not extract velocity command");
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    return false;
  }

  // Apply acceleration limits (from teb_config)
  double current_v = robot_vel.pose.position.x;
  double current_w = tf2::getYaw(robot_vel.pose.orientation);

  double max_linear_acc = teb_optimizer_->getConfig().max_linear_acc;
  double max_angular_acc = teb_optimizer_->getConfig().max_angular_acc;

  // Rate-limited velocity commands
  double v_lo = current_v - max_linear_acc * 0.1;
  double v_hi = current_v + max_linear_acc * 0.1;
  cmd_vel.linear.x = std::max(v_lo, std::min(v, v_hi));
  double w_lo = current_w - max_angular_acc * 0.1;
  double w_hi = current_w + max_angular_acc * 0.1;
  cmd_vel.angular.z = std::max(w_lo, std::min(w, w_hi));

  // Publish visualization
  std::vector<geometry_msgs::PoseStamped> local_plan;
  teb_optimizer_->getTrajectoryAsPath(local_plan);
  publishLocalPlan(local_plan);
  publishGlobalPlan(prune_plan);

  // Publish TEB trajectory for debugging
  nav_msgs::Path teb_path;
  teb_path.header.frame_id = costmap_ros_->getGlobalFrameID();
  teb_path.header.stamp = ros::Time::now();
  teb_path.poses = local_plan;
  teb_trajectory_pub_.publish(teb_path);

  return true;
}

bool TEBController::isGoalReached() {
  if (!isInitialized()) {
    ROS_ERROR("TEB Controller has not been initialized");
    return false;
  }

  if (!costmap_ros_->getRobotPose(current_pose_)) {
    ROS_ERROR("TEB: Could not get robot pose");
    return false;
  }

  // Only check goal with latched controller if plan has valid frame_ids
  bool have_valid_plan =
      !global_plan_.empty() && !global_plan_.back().header.frame_id.empty();
  if (!have_valid_plan) {
    // Fallback: simple distance + angle check
    double dx = goal_x_ - current_pose_.pose.position.x;
    double dy = goal_y_ - current_pose_.pose.position.y;
    double dist = std::hypot(dx, dy);
    double angle_diff = angles::shortest_angular_distance(
        tf2::getYaw(current_pose_.pose.orientation), goal_theta_);
    if (dist < 0.3 && std::abs(angle_diff) < 0.3) {
      goal_reached_ = true;
      return true;
    }
    return false;
  }

  if (latched_stop_rotate_controller_.isGoalReached(
          &planner_util_, odom_helper_, current_pose_)) {
    if (!goal_reached_) {
      R_INFO << "TEB: GOAL Reached!";
      goal_reached_ = true;
    }
    return true;
  }

  return false;
}

void TEBController::publishLocalPlan(
    const std::vector<geometry_msgs::PoseStamped> &path) {
  base_local_planner::publishPlan(path, l_plan_pub_);
}

void TEBController::publishGlobalPlan(
    const std::vector<geometry_msgs::PoseStamped> &path) {
  base_local_planner::publishPlan(path, g_plan_pub_);
}

void TEBController::transformPose(const std::string &target_frame,
                                  const geometry_msgs::PoseStamped &in_pose,
                                  geometry_msgs::PoseStamped &out_pose) const {
  // Avoid TF lookup with empty frame_ids (causes tf2 warnings)
  if (tf_ && !target_frame.empty() && !in_pose.header.frame_id.empty() &&
      tf_->canTransform(target_frame, in_pose.header.frame_id, ros::Time(0))) {
    tf_->transform(in_pose, out_pose, target_frame);
  } else {
    out_pose = in_pose;
  }
}

std::vector<geometry_msgs::PoseStamped>
TEBController::prunePlan(const geometry_msgs::PoseStamped &robot_pose) const {
  if (global_plan_.empty()) {
    return {};
  }

  // Find the nearest point on the path to the robot
  size_t nearest_idx = 0;
  double min_dist = std::numeric_limits<double>::max();

  for (size_t i = 0; i < global_plan_.size(); ++i) {
    double dx = global_plan_[i].pose.position.x - robot_pose.pose.position.x;
    double dy = global_plan_[i].pose.position.y - robot_pose.pose.position.y;
    double dist = dx * dx + dy * dy;
    if (dist < min_dist) {
      min_dist = dist;
      nearest_idx = i;
    }
  }

  // Keep from nearest_idx+1 onwards (skip the point the robot has already
  // passed). This prevents the look-ahead from pointing backward when the robot
  // is close to or past a waypoint.
  std::vector<geometry_msgs::PoseStamped> pruned;
  for (size_t i = nearest_idx + 1; i < global_plan_.size(); ++i) {
    pruned.push_back(global_plan_[i]);
  }

  // If pruning removed everything (robot at or past the last waypoint),
  // include the last waypoint as the goal reference.
  if (pruned.empty() && !global_plan_.empty()) {
    pruned.push_back(global_plan_.back());
  }

  return pruned;
}

} // namespace rmp::controller
