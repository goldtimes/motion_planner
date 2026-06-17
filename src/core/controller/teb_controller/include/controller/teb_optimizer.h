/**
 * *********************************************************
 *
 * @file: teb_optimizer.h
 * @brief: Contains the TEB optimizer using Ceres-solver for local trajectory
 * optimization
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
#ifndef RMP_CONTROLLER_TEB_OPTIMIZER_H_
#define RMP_CONTROLLER_TEB_OPTIMIZER_H_

#include <Eigen/Core>
#include <memory>
#include <vector>

#include <ceres/ceres.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include "common/geometry/point.h"

namespace rmp::controller {

/**
 * @brief Configuration for TEB optimizer
 */
struct TEBConfig {
  // optimization horizon
  int teb_horizon = 30;           // number of poses in TEB trajectory
  double teb_dt_resolution = 0.3; // base time resolution (s)
  int teb_iterations = 100;       // max optimization iterations

  // weight coefficients
  double weight_path = 1.0;         // path following weight
  double weight_obstacle = 1.0;     // obstacle avoidance weight
  double weight_velocity = 1.0;     // velocity limits weight
  double weight_acceleration = 1.0; // acceleration limits weight
  double weight_time = 1.0;         // time optimality weight
  double weight_smoothness = 1.0;   // trajectory smoothness weight
  double weight_kinematics = 1.0;   // non-holonomic kinematics weight

  // velocity limits
  double max_linear_vel = 0.5;
  double min_linear_vel = 0.0;
  double max_angular_vel = 1.5;
  double min_angular_vel = 0.0;

  // acceleration limits
  double max_linear_acc = 0.5;
  double max_angular_acc = 1.5;

  // obstacle avoidance
  double obstacle_min_dist = 0.2;       // minimum distance to obstacles
  double inflate_obstacle_radius = 0.5; // inflation radius for obstacles

  // optimization parameters
  double convergence_delta = 1e-3;
  double dt_hysteresis = 0.1;
};

/**
 * @brief A single pose in the TEB trajectory (SE2 + time)
 */
struct TEBPose {
  double x = 0.0;     // x position (m)
  double y = 0.0;     // y position (m)
  double theta = 0.0; // orientation (rad)
  double dt = 0.3;    // time to next pose (s)

  TEBPose() = default;
  TEBPose(double x, double y, double theta, double dt)
      : x(x), y(y), theta(theta), dt(dt) {}
};

/**
 * @brief Timed Elastic Band optimizer using Ceres-solver
 *
 * Implements a TEB local trajectory planner that optimizes a sequence of
 * robot poses (x, y, theta) with associated time durations (dt) using
 * Ceres-solver for sparse nonlinear optimization.
 */
class TEBOptimizer {
public:
  TEBOptimizer();
  ~TEBOptimizer() = default;

  /**
   * @brief Initialize the optimizer with configuration
   * @param cfg  TEB configuration
   * @param costmap  pointer to costmap
   */
  void initialize(const TEBConfig &cfg, costmap_2d::Costmap2D *costmap);

  /**
   * @brief Set the reference global plan
   */
  void
  setReferencePlan(const std::vector<geometry_msgs::PoseStamped> &global_plan);

  /**
   * @brief Initialize the TEB trajectory from current pose and reference plan
   */
  void initTrajectory(const Eigen::Vector3d &current_pose,
                      const std::vector<geometry_msgs::PoseStamped> &ref_plan);

  /**
   * @brief Build and solve the Ceres optimization problem
   * @return true if optimization succeeded
   */
  bool optimize();

  /**
   * @brief Get the optimized trajectory
   */
  const std::vector<TEBPose> &getTrajectory() const;

  /**
   * @brief Get the optimized trajectory as a path of PoseStamped
   */
  void getTrajectoryAsPath(std::vector<geometry_msgs::PoseStamped> &path) const;

  /**
   * @brief Extract velocity commands from the optimized trajectory
   * @param v  output linear velocity (m/s)
   * @param w  output angular velocity (rad/s)
   * @return true if commands are valid
   */
  bool getVelocityCommand(double &v, double &w) const;

  /**
   * @brief Check if the goal has been reached
   */
  bool isGoalReached() const;

  /**
   * @brief Set the goal pose
   */
  void setGoal(double goal_x, double goal_y, double goal_theta);

  /**
   * @brief Resize the trajectory to the desired horizon
   */
  void resizeTrajectory(int new_size);

  /**
   * @brief Get reference path index for the current pose
   */
  int findReferenceIndex(const Eigen::Vector3d &pose) const;

private:
  /**
   * @brief Compute obstacle cost at a given point using costmap
   */
  double getObstacleCost(double x, double y) const;

  /**
   * @brief Interpolate reference path to get a desired pose
   */
  Eigen::Vector3d interpolateReference(int idx, double t) const;

private:
  TEBConfig config_;
  costmap_2d::Costmap2D *costmap_{nullptr};

  // TEB trajectory: sequence of poses with time deltas
  std::vector<TEBPose> trajectory_;

  // Reference global plan (stored as x, y, theta)
  std::vector<Eigen::Vector3d> reference_plan_;
  std::vector<double> reference_arclength_;

  // Goal pose
  Eigen::Vector3d goal_pose_{0, 0, 0};

  // Optimization problem
  std::unique_ptr<ceres::Problem> problem_;
  std::unique_ptr<ceres::Solver::Options> solver_options_;
  ceres::Solver::Summary summary_;

  // Parameter blocks for Ceres (raw pointers for ownership)
  std::vector<double *> param_blocks_;

  // Reference index tracking
  int reference_idx_{0};

  // Previous velocity commands (for acceleration calculation)
  double prev_v_{0.0};
  double prev_w_{0.0};
};

// ============================================================
// Ceres cost functors
//
// Each functor uses AutoDiffCostFunction with known block sizes.
// A TEB pose parameter block is [x, y, theta, dt] (size 4).
// ============================================================

/**
 * @brief Path following cost: penalize deviation from reference path
 *
 * Takes 1 parameter block [x, y, theta, dt]
 * Residual: [weight * pos_error^2, weight * angle_error^2]
 */
class PathFollowingCostFunctor {
public:
  PathFollowingCostFunctor(double weight, const Eigen::Vector3d &ref_pose)
      : weight_(weight), ref_x_(ref_pose.x()), ref_y_(ref_pose.y()),
        ref_theta_(ref_pose.z()) {}

  template <typename T>
  bool operator()(const T *const pose, T *residual) const {
    T dx = pose[0] - T(ref_x_);
    T dy = pose[1] - T(ref_y_);
    T dtheta = pose[2] - T(ref_theta_);
    dtheta = ceres::atan2(ceres::sin(dtheta), ceres::cos(dtheta));

    residual[0] = T(weight_) * (dx * dx + dy * dy);
    residual[1] = T(weight_) * (dtheta * dtheta);
    return true;
  }

private:
  double weight_;
  double ref_x_, ref_y_, ref_theta_;
};

/**
 * @brief Obstacle avoidance cost (auto-diff compatible approximation)
 *
 * Uses a pre-computed obstacle cost from the costmap.
 * Takes 1 parameter block [x, y, theta, dt]
 * Residual: weight * obstacle_penalty
 */
class ObstacleCostFunctor {
public:
  ObstacleCostFunctor(double weight, double obstacle_cost)
      : weight_(weight), obstacle_cost_(obstacle_cost) {}

  template <typename T>
  bool operator()(const T *const pose, T *residual) const {
    (void)pose;
    residual[0] = T(weight_) * T(obstacle_cost_);
    return true;
  }

private:
  double weight_;
  double obstacle_cost_;
};

/**
 * @brief Velocity constraints: ensure feasible velocity between consecutive
 * poses
 *
 * Takes 2 parameter blocks: pose_i [x,y,theta,dt], pose_i+1 [x,y,theta,dt]
 * v = dist / dt_i, w = dtheta / dt_i
 * Residual: [weight * max(0, |v|-v_max), weight * max(0, |w|-w_max)]
 */
class VelocityCostFunctor {
public:
  VelocityCostFunctor(double weight, double v_max, double v_min, double w_max)
      : weight_(weight), v_max_(v_max), v_min_(v_min), w_max_(w_max) {}

  template <typename T>
  bool operator()(const T *const pose_i, const T *const pose_ip1,
                  T *residual) const {
    T dx = pose_ip1[0] - pose_i[0];
    T dy = pose_ip1[1] - pose_i[1];
    T dt = pose_i[3] + T(1e-6);

    T dist = ceres::sqrt(dx * dx + dy * dy);
    T v = dist / dt;

    T dtheta = pose_ip1[2] - pose_i[2];
    dtheta = ceres::atan2(ceres::sin(dtheta), ceres::cos(dtheta));
    T w = dtheta / dt;

    T v_err = T(0.0);
    T v_abs = ceres::abs(v);
    if (v_abs > T(v_max_)) {
      v_err = v_abs - T(v_max_);
    } else if (v_abs < T(v_min_) && v_abs > T(0.01)) {
      v_err = T(v_min_) - v_abs;
    }

    T w_err = T(0.0);
    T w_abs = ceres::abs(w);
    if (w_abs > T(w_max_)) {
      w_err = w_abs - T(w_max_);
    }

    residual[0] = T(weight_) * v_err;
    residual[1] = T(weight_) * w_err;
    return true;
  }

private:
  double weight_;
  double v_max_, v_min_, w_max_;
};

/**
 * @brief Time optimality cost: minimize total trajectory time
 *
 * Takes 1 parameter block [x, y, theta, dt]
 * Residual: weight * dt
 */
class TimeCostFunctor {
public:
  TimeCostFunctor(double weight) : weight_(weight) {}

  template <typename T>
  bool operator()(const T *const pose, T *residual) const {
    residual[0] = T(weight_) * pose[3]; // dt
    return true;
  }

private:
  double weight_;
};

/**
 * @brief Smoothness cost: penalize changes in consecutive pose transitions
 *
 * Takes 3 parameter blocks: pose_i, pose_i+1, pose_i+2
 * Residual: [weight * (dx2-dx1), weight * (dy2-dy1), weight *
 * (dtheta2-dtheta1)]
 */
class SmoothnessCostFunctor {
public:
  SmoothnessCostFunctor(double weight) : weight_(weight) {}

  template <typename T>
  bool operator()(const T *const pose_i, const T *const pose_ip1,
                  const T *const pose_ip2, T *residual) const {
    T dx1 = pose_ip1[0] - pose_i[0];
    T dy1 = pose_ip1[1] - pose_i[1];
    T dx2 = pose_ip2[0] - pose_ip1[0];
    T dy2 = pose_ip2[1] - pose_ip1[1];

    T dtheta1 = pose_ip1[2] - pose_i[2];
    dtheta1 = ceres::atan2(ceres::sin(dtheta1), ceres::cos(dtheta1));
    T dtheta2 = pose_ip2[2] - pose_ip1[2];
    dtheta2 = ceres::atan2(ceres::sin(dtheta2), ceres::cos(dtheta2));

    residual[0] = T(weight_) * (dx2 - dx1);
    residual[1] = T(weight_) * (dy2 - dy1);
    residual[2] = T(weight_) * (dtheta2 - dtheta1);
    return true;
  }

private:
  double weight_;
};

/**
 * @brief Non-holonomic kinematics cost
 *
 * Penalizes deviation from the non-holonomic constraint:
 * the robot's heading should align with the motion direction.
 *
 * Takes 2 parameter blocks: pose_i, pose_i+1
 * Residual: [weight * dist * heading_error_i, weight * dist *
 * heading_error_i+1]
 */
class KinematicsCostFunctor {
public:
  KinematicsCostFunctor(double weight) : weight_(weight) {}

  template <typename T>
  bool operator()(const T *const pose_i, const T *const pose_ip1,
                  T *residual) const {
    T dx = pose_ip1[0] - pose_i[0];
    T dy = pose_ip1[1] - pose_i[1];
    T dist = ceres::sqrt(dx * dx + dy * dy);

    T motion_heading = ceres::atan2(dy, dx);

    T heading_err = pose_i[2] - motion_heading;
    heading_err =
        ceres::atan2(ceres::sin(heading_err), ceres::cos(heading_err));

    T heading_err_next = pose_ip1[2] - motion_heading;
    heading_err_next = ceres::atan2(ceres::sin(heading_err_next),
                                    ceres::cos(heading_err_next));

    residual[0] = T(weight_) * dist * heading_err;
    residual[1] = T(weight_) * dist * heading_err_next;
    return true;
  }

private:
  double weight_;
};

} // namespace rmp::controller

#endif // RMP_CONTROLLER_TEB_OPTIMIZER_H_
