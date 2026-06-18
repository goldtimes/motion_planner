/**
 * *********************************************************
 *
 * @file: teb_optimizer.cpp
 * @brief: Implements the TEB optimizer using Ceres-solver
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
#include <algorithm>
#include <cmath>
#include <limits>

#include <ros/console.h>
#include <tf2/utils.h>

#include <costmap_2d/cost_values.h>

#include "common/util/log.h"
#include "controller/teb_optimizer.h"

namespace rmp::controller {

TEBOptimizer::TEBOptimizer() : costmap_(nullptr) {}

void TEBOptimizer::initialize(const TEBConfig &cfg,
                              costmap_2d::Costmap2D *costmap) {
  config_ = cfg;
  costmap_ = costmap;

  // Setup solver options
  solver_options_ = std::make_unique<ceres::Solver::Options>();
  solver_options_->max_num_iterations = config_.teb_iterations;
  solver_options_->function_tolerance = config_.convergence_delta;
  solver_options_->gradient_tolerance = config_.convergence_delta;
  solver_options_->parameter_tolerance = config_.convergence_delta;
  solver_options_->minimizer_progress_to_stdout = false;
  solver_options_->num_threads = 1;
  solver_options_->linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  solver_options_->trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;

  R_INFO << "TEB Optimizer initialized with horizon=" << config_.teb_horizon
         << ", dt=" << config_.teb_dt_resolution;
}

void TEBOptimizer::setReferencePlan(
    const std::vector<geometry_msgs::PoseStamped> &global_plan) {
  reference_plan_.clear();
  reference_arclength_.clear();

  if (global_plan.empty()) {
    return;
  }

  double arclength = 0.0;
  reference_arclength_.push_back(arclength);

  Eigen::Vector3d prev_pose;
  for (size_t i = 0; i < global_plan.size(); ++i) {
    const auto &p = global_plan[i];
    double theta = tf2::getYaw(p.pose.orientation);
    Eigen::Vector3d pose(p.pose.position.x, p.pose.position.y, theta);
    reference_plan_.push_back(pose);

    if (i > 0) {
      double dx = pose.x() - prev_pose.x();
      double dy = pose.y() - prev_pose.y();
      arclength += std::sqrt(dx * dx + dy * dy);
      reference_arclength_.push_back(arclength);
    }
    prev_pose = pose;
  }

  R_DEBUG << "TEB: Reference plan loaded with " << reference_plan_.size()
          << " waypoints, "
          << "length=" << arclength;
}

void TEBOptimizer::initTrajectory(
    const Eigen::Vector3d &current_pose,
    const std::vector<geometry_msgs::PoseStamped> &ref_plan) {
  trajectory_.clear();

  if (ref_plan.empty()) {
    // Create a minimal forward trajectory from current pose
    for (int i = 0; i < config_.teb_horizon; ++i) {
      double t = static_cast<double>(i) * config_.teb_dt_resolution;
      TEBPose pose;
      pose.x = current_pose.x() + t * config_.max_linear_vel * 0.5;
      pose.y = current_pose.y();
      pose.theta = current_pose.z();
      pose.dt = config_.teb_dt_resolution;
      trajectory_.push_back(pose);
    }
    return;
  }

  // Initialize TEB by sampling the reference plan
  reference_idx_ = findReferenceIndex(current_pose);

  // Total arc length remaining
  double total_length =
      reference_arclength_.back() - reference_arclength_[reference_idx_];

  // If the remaining path is too short, create a minimal forward trajectory
  // to avoid collapsing all poses to the same point (which causes NaN in
  // Ceres).
  if (total_length < 0.3) {
    trajectory_.clear();
    for (int i = 0; i < config_.teb_horizon; ++i) {
      double t = static_cast<double>(i) * config_.teb_dt_resolution;
      TEBPose pose;
      pose.x = current_pose.x() +
               t * config_.max_linear_vel * 0.5 * std::cos(current_pose.z());
      pose.y = current_pose.y() +
               t * config_.max_linear_vel * 0.5 * std::sin(current_pose.z());
      pose.theta = current_pose.z();
      pose.dt = config_.teb_dt_resolution;
      trajectory_.push_back(pose);
    }
    // Ensure first pose matches current robot pose
    trajectory_[0].x = current_pose.x();
    trajectory_[0].y = current_pose.y();
    trajectory_[0].theta = current_pose.z();
    R_DEBUG << "TEB: Short path (" << total_length
            << "m), using forward projection";
    return;
  }

  // Distribute poses along the reference path
  double step = total_length / static_cast<double>(config_.teb_horizon);

  for (int i = 0; i < config_.teb_horizon; ++i) {
    double target_arclength =
        reference_arclength_[reference_idx_] + static_cast<double>(i) * step;
    if (target_arclength > reference_arclength_.back()) {
      target_arclength = reference_arclength_.back();
    }

    // Find the segment containing this arc length
    TEBPose pose;
    for (size_t j = reference_idx_; j < reference_arclength_.size() - 1; ++j) {
      if (reference_arclength_[j] <= target_arclength &&
          target_arclength <= reference_arclength_[j + 1]) {
        double seg_len = reference_arclength_[j + 1] - reference_arclength_[j];
        double t = (seg_len > 0)
                       ? (target_arclength - reference_arclength_[j]) / seg_len
                       : 0.0;
        t = std::max(0.0, std::min(t, 1.0));

        pose.x = reference_plan_[j].x() +
                 t * (reference_plan_[j + 1].x() - reference_plan_[j].x());
        pose.y = reference_plan_[j].y() +
                 t * (reference_plan_[j + 1].y() - reference_plan_[j].y());
        pose.theta = reference_plan_[j].z() +
                     t * (reference_plan_[j + 1].z() - reference_plan_[j].z());
        break;
      }
    }
    pose.dt = config_.teb_dt_resolution;

    // Obstacle-aware initialization: if pose is inside an obstacle,
    // shift it sideways (perpendicular to path direction) to find free space.
    const int max_shift_attempts = 10;
    const double shift_step = costmap_ ? costmap_->getResolution() * 2.0 : 0.1;
    double occ = getObstacleCost(pose.x, pose.y);
    if (occ > 0.5) {
      // Determine shift direction from the segment heading
      double heading = pose.theta;
      // Try left shifts, then right shifts
      for (int s = 1; s <= max_shift_attempts; ++s) {
        double shift = s * shift_step;
        // Try left
        double lx = pose.x + shift * std::cos(heading + M_PI / 2.0);
        double ly = pose.y + shift * std::sin(heading + M_PI / 2.0);
        if (getObstacleCost(lx, ly) < 0.2) {
          pose.x = lx;
          pose.y = ly;
          break;
        }
        // Try right
        double rx = pose.x + shift * std::cos(heading - M_PI / 2.0);
        double ry = pose.y + shift * std::sin(heading - M_PI / 2.0);
        if (getObstacleCost(rx, ry) < 0.2) {
          pose.x = rx;
          pose.y = ry;
          break;
        }
      }
    }
    trajectory_.push_back(pose);
  }

  // Ensure first pose matches current robot pose
  if (!trajectory_.empty()) {
    trajectory_[0].x = current_pose.x();
    trajectory_[0].y = current_pose.y();
    trajectory_[0].theta = current_pose.z();

    // Align p1 with p0's heading direction so the kinematics constraint
    // (p0.theta = motion_dir) can be satisfied from the start.
    // Without this, p1 is placed on the reference path which may be to the
    // side of the robot, producing dtheta=0 but requiring sideways motion.
    if (trajectory_.size() > 1) {
      double dist = std::hypot(trajectory_[1].x - trajectory_[0].x,
                               trajectory_[1].y - trajectory_[0].y);
      trajectory_[1].x =
          trajectory_[0].x + dist * std::cos(trajectory_[0].theta);
      trajectory_[1].y =
          trajectory_[0].y + dist * std::sin(trajectory_[0].theta);
      // Keep p1.theta as-is (it will be optimized to match the curve)
    }
  }

  R_DEBUG << "TEB: Trajectory initialized with " << trajectory_.size()
          << " poses";
}

bool TEBOptimizer::optimize() {
  if (trajectory_.size() < 3) {
    R_WARN << "TEB: Trajectory too short for optimization";
    return false;
  }

  // Build Ceres problem
  problem_ = std::make_unique<ceres::Problem>();
  param_blocks_.clear();

  int N = static_cast<int>(trajectory_.size());

  // Add parameter blocks for each pose
  // Each pose has 4 parameters: [x, y, theta, dt]
  for (int i = 0; i < N; ++i) {
    auto &pose = trajectory_[i];
    double *block = new double[4]{pose.x, pose.y, pose.theta, pose.dt};
    param_blocks_.push_back(block);
    problem_->AddParameterBlock(block, 4);

    // Fix the first pose (robot's current position)
    if (i == 0) {
      problem_->SetParameterBlockConstant(block);
    }
  }

  // -------------------------------------------------------
  // Add cost functions
  // -------------------------------------------------------

  // 1. Path following cost: each pose should be close to the reference path
  for (int i = 0; i < N; ++i) {
    // Find nearest reference point for this pose
    double target_x = trajectory_[i].x;
    double target_y = trajectory_[i].y;
    double target_theta = trajectory_[i].theta;

    double min_dist = std::numeric_limits<double>::max();
    for (const auto &ref : reference_plan_) {
      double dx = ref.x() - param_blocks_[i][0];
      double dy = ref.y() - param_blocks_[i][1];
      double dist = dx * dx + dy * dy;
      if (dist < min_dist) {
        min_dist = dist;
        target_x = ref.x();
        target_y = ref.y();
        target_theta = ref.z();
      }
    }

    ceres::CostFunction *path_cost =
        new ceres::AutoDiffCostFunction<PathFollowingCostFunctor, 2, 4>(
            new PathFollowingCostFunctor(
                config_.weight_path,
                Eigen::Vector3d(target_x, target_y, target_theta)));
    problem_->AddResidualBlock(path_cost, nullptr, param_blocks_[i]);
  }

  // 2. Velocity cost between consecutive poses
  for (int i = 0; i < N - 1; ++i) {
    ceres::CostFunction *vel_cost =
        new ceres::AutoDiffCostFunction<VelocityCostFunctor, 2, 4, 4>(
            new VelocityCostFunctor(
                config_.weight_velocity, config_.max_linear_vel,
                config_.min_linear_vel, config_.max_angular_vel));
    problem_->AddResidualBlock(vel_cost, nullptr, param_blocks_[i],
                               param_blocks_[i + 1]);
  }

  // 3. Time optimality cost (minimize dt for each pose)
  for (int i = 0; i < N; ++i) {
    ceres::CostFunction *time_cost =
        new ceres::AutoDiffCostFunction<TimeCostFunctor, 1, 4>(
            new TimeCostFunctor(config_.weight_time));
    problem_->AddResidualBlock(time_cost, nullptr, param_blocks_[i]);
  }

  // 4. Smoothness cost (triplets of consecutive poses)
  for (int i = 0; i < N - 2; ++i) {
    ceres::CostFunction *smooth_cost =
        new ceres::AutoDiffCostFunction<SmoothnessCostFunctor, 3, 4, 4, 4>(
            new SmoothnessCostFunctor(config_.weight_smoothness));
    problem_->AddResidualBlock(smooth_cost, nullptr, param_blocks_[i],
                               param_blocks_[i + 1], param_blocks_[i + 2]);
  }

  // 5. Non-holonomic kinematics cost
  for (int i = 0; i < N - 1; ++i) {
    ceres::CostFunction *kin_cost =
        new ceres::AutoDiffCostFunction<KinematicsCostFunctor, 1, 4, 4>(
            new KinematicsCostFunctor(config_.weight_kinematics));
    problem_->AddResidualBlock(kin_cost, nullptr, param_blocks_[i],
                               param_blocks_[i + 1]);
  }

  // 6. Obstacle cost (using costmap — pre-computed in per_pose_obstacle_cost_)
  for (int i = 0; i < N; ++i) {
    double occ_cost = (i < static_cast<int>(per_pose_obstacle_cost_.size()))
                          ? per_pose_obstacle_cost_[i]
                          : getObstacleCost(trajectory_[i].x, trajectory_[i].y);
    if (occ_cost > 1e-6) {
      ceres::CostFunction *obs_cost =
          new ceres::AutoDiffCostFunction<ObstacleCostFunctor, 1, 4>(
              new ObstacleCostFunctor(config_.weight_obstacle, occ_cost));
      problem_->AddResidualBlock(obs_cost, nullptr, param_blocks_[i]);
    }
  }

  // Solve
  ceres::Solver::Options options = *solver_options_;
  ceres::Solve(options, problem_.get(), &summary_);

  bool success = summary_.termination_type != ceres::FAILURE;

  if (success) {
    // Extract optimized values back to trajectory
    for (int i = 0; i < N; ++i) {
      trajectory_[i].x = param_blocks_[i][0];
      trajectory_[i].y = param_blocks_[i][1];
      trajectory_[i].theta = param_blocks_[i][2];
      trajectory_[i].dt =
          std::max(param_blocks_[i][3], 0.01); // ensure positive dt
    }

    R_DEBUG << "TEB: Optimization succeeded, "
            << "iterations=" << summary_.iterations.size()
            << ", final_cost=" << summary_.final_cost;
  } else {
    R_WARN << "TEB: Optimization failed: " << summary_.message;
  }

  // Clean up parameter blocks
  for (auto *block : param_blocks_) {
    delete[] block;
  }
  param_blocks_.clear();

  return success;
}

void TEBOptimizer::updateObstacleCosts() {
  // Re-compute obstacle costs for each trajectory pose based on current
  // positions. This is called between optimization passes so that the next pass
  // sees updated obstacle costs (approximating a gradient for avoidance).
  per_pose_obstacle_cost_.resize(trajectory_.size());
  for (size_t i = 0; i < trajectory_.size(); ++i) {
    per_pose_obstacle_cost_[i] =
        getObstacleCost(trajectory_[i].x, trajectory_[i].y);
  }
}

const std::vector<TEBPose> &TEBOptimizer::getTrajectory() const {
  return trajectory_;
}

void TEBOptimizer::getTrajectoryAsPath(
    std::vector<geometry_msgs::PoseStamped> &path) const {
  path.clear();
  for (const auto &pose : trajectory_) {
    geometry_msgs::PoseStamped ps;
    ps.pose.position.x = pose.x;
    ps.pose.position.y = pose.y;
    ps.pose.position.z = 0.0;

    // Convert theta to quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, pose.theta);
    ps.pose.orientation.x = q.x();
    ps.pose.orientation.y = q.y();
    ps.pose.orientation.z = q.z();
    ps.pose.orientation.w = q.w();

    path.push_back(ps);
  }
}

bool TEBOptimizer::getVelocityCommand(double &v, double &w) const {
  if (trajectory_.size() < 2) {
    v = 0.0;
    w = 0.0;
    return false;
  }

  const auto &p0 = trajectory_[0];
  const auto &p1 = trajectory_[1];

  double dx = p1.x - p0.x;
  double dy = p1.y - p0.y;
  double dist = std::sqrt(dx * dx + dy * dy);
  double dt = std::max(p0.dt, 1e-3);

  // Linear velocity from local displacement
  v = dist / dt;

  // ── Angular velocity: use a look-ahead point on the reference plan ──
  // Instead of computing w from p1.theta - p0.theta (which is noisy because
  // the optimizer's local p1 position oscillates between kinematics and
  // path costs), compute the heading error towards a look-ahead point
  // on the reference path. This gives smooth, stable turning.
  double desired_heading = p0.theta;

  if (!reference_plan_.empty()) {
    double lookahead_dist = 0.5;
    double accumulated = 0.0;
    Eigen::Vector3d lookahead_pt = reference_plan_.back();

    for (size_t i = reference_idx_; i + 1 < reference_plan_.size(); ++i) {
      double seg_dx = reference_plan_[i + 1].x() - reference_plan_[i].x();
      double seg_dy = reference_plan_[i + 1].y() - reference_plan_[i].y();
      double seg_len = std::sqrt(seg_dx * seg_dx + seg_dy * seg_dy);
      accumulated += seg_len;
      if (accumulated >= lookahead_dist) {
        lookahead_pt = reference_plan_[i + 1];
        break;
      }
    }

    desired_heading =
        std::atan2(lookahead_pt.y() - p0.y, lookahead_pt.x() - p0.x);
  }

  double heading_err = desired_heading - p0.theta;
  heading_err = std::atan2(std::sin(heading_err), std::cos(heading_err));

  // P-controller: w = gain * heading_error
  // gain=2.0 → 90° error → w≈3.1 rad/s, 10° error → w≈0.35 rad/s
  double w_gain = 2.0;
  w = w_gain * heading_err;

  // Clamp to limits
  v = std::max(config_.min_linear_vel, std::min(v, config_.max_linear_vel));
  w = std::max(config_.min_angular_vel, std::min(w, config_.max_angular_vel));

  // Direction check
  double motion_dir = std::atan2(dy, dx);
  double heading_diff = motion_dir - p0.theta;
  heading_diff = std::atan2(std::sin(heading_diff), std::cos(heading_diff));
  if (std::abs(heading_diff) > M_PI / 2.0) {
    v = -v;
  }

  // Debug log
  static int debug_counter = 0;
  if (++debug_counter % 10 == 1) {
    ROS_INFO("TEB vel: p0=(%.2f,%.2f,th=%.2f) v=%.3f w=%.3f "
             "des_hdg=%.2f err=%.2f ref_pts=%zu",
             p0.x, p0.y, p0.theta, v, w, desired_heading, heading_err,
             reference_plan_.size());
  }

  return true;
}

bool TEBOptimizer::isGoalReached() const {
  if (trajectory_.empty()) {
    return false;
  }

  const auto &last_pose = trajectory_.back();
  double dx = last_pose.x - goal_pose_.x();
  double dy = last_pose.y - goal_pose_.y();
  double dtheta = last_pose.theta - goal_pose_.z();
  dtheta = std::atan2(std::sin(dtheta), std::cos(dtheta));

  // Use 2x the config tolerance (since TEB poses are discretized)
  const double dist_tol = 0.3;
  const double angle_tol = 0.5;

  return (dx * dx + dy * dy < dist_tol * dist_tol) &&
         (std::abs(dtheta) < angle_tol);
}

void TEBOptimizer::setGoal(double goal_x, double goal_y, double goal_theta) {
  goal_pose_ = Eigen::Vector3d(goal_x, goal_y, goal_theta);
}

void TEBOptimizer::resizeTrajectory(int new_size) {
  if (new_size < 3) {
    new_size = 3;
  }

  int old_size = static_cast<int>(trajectory_.size());

  if (new_size > old_size) {
    // Extend by duplicating last pose
    for (int i = old_size; i < new_size; ++i) {
      trajectory_.push_back(trajectory_.back());
    }
  } else if (new_size < old_size) {
    trajectory_.resize(new_size);
  }
}

int TEBOptimizer::findReferenceIndex(const Eigen::Vector3d &pose) const {
  if (reference_plan_.empty()) {
    return 0;
  }

  int nearest_idx = 0;
  double min_dist = std::numeric_limits<double>::max();

  for (size_t i = 0; i < reference_plan_.size(); ++i) {
    double dx = reference_plan_[i].x() - pose.x();
    double dy = reference_plan_[i].y() - pose.y();
    double dist = dx * dx + dy * dy;
    if (dist < min_dist) {
      min_dist = dist;
      nearest_idx = static_cast<int>(i);
    }
  }

  return nearest_idx;
}

double TEBOptimizer::getObstacleCost(double wx, double wy) const {
  if (!costmap_) {
    return 0.0;
  }

  unsigned int mx, my;
  if (!costmap_->worldToMap(wx, wy, mx, my)) {
    return 0.5; // outside map: penalize mildly (conservative)
  }

  unsigned char cost = costmap_->getCost(mx, my);
  if (cost == costmap_2d::NO_INFORMATION) {
    return 0.1; // unknown: small penalty (conservative)
  }

  // Normalize cost to [0, 1]
  double normalized = static_cast<double>(cost) / 255.0;

  // Exponential scaling: cost grows rapidly as you approach an obstacle.
  // At cost=0 (free) → 0
  // At cost=200 (~0.78) → mild
  // At cost=253 (INSCRIBED_INFLATED) → large
  if (normalized > 0.3) {
    double x = (normalized - 0.3) / 0.7; // remap [0.3, 1.0] → [0.0, 1.0]
    return std::exp(4.0 * x) - 1.0;
  }

  return 0.0;
}

Eigen::Vector3d TEBOptimizer::interpolateReference(int idx, double t) const {
  if (reference_plan_.empty()) {
    return Eigen::Vector3d::Zero();
  }

  int max_idx = static_cast<int>(reference_plan_.size()) - 1;
  int idx1 = std::max(0, std::min(idx, max_idx));
  int idx2 = std::max(0, std::min(idx + 1, max_idx));
  t = std::max(0.0, std::min(t, 1.0));

  Eigen::Vector3d result;
  result.x() = reference_plan_[idx1].x() +
               t * (reference_plan_[idx2].x() - reference_plan_[idx1].x());
  result.y() = reference_plan_[idx1].y() +
               t * (reference_plan_[idx2].y() - reference_plan_[idx1].y());
  result.z() = reference_plan_[idx1].z() +
               t * (reference_plan_[idx2].z() - reference_plan_[idx1].z());

  return result;
}

} // namespace rmp::controller
