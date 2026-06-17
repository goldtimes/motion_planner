/**
 * *********************************************************
 *
 * @file: my_dwa_controller.cpp
 * @brief: Implementation of a from-scratch Dynamic Window Approach (DWA)
 *         local planner.
 *
 * DWA Algorithm Steps (called every control cycle):
 *
 *   Step 1 — Get current state
 *     Read robot pose (x, y, theta) from costmap and velocity (vx, vy, vth)
 *     from odometry.
 *
 *   Step 2 — Compute the Dynamic Window
 *     The dynamic window is the intersection of three constraints:
 *       a) Velocity limits:    v ∈ [v_min, v_max],  w ∈ [w_min, w_max]
 *       b) Acceleration limits: reachable velocities given current velocity
 *          and max acceleration over one control cycle:
 *            v ∈ [v_curr - acc_x * dt,  v_curr + acc_x * dt]
 *            w ∈ [w_curr - acc_th * dt, w_curr + acc_th * dt]
 *       c) Braking condition: velocities from which the robot can stop
 *          before hitting the nearest obstacle.
 *
 *   Step 3 — Sample velocities
 *     Discretize the dynamic window into a grid of (v, w) samples.
 *
 *   Step 4 — Predict trajectories
 *     For each (v, w) pair, simulate forward using the robot's motion model
 *     for sim_time seconds, recording the resulting path of poses.
 *
 *   Step 5 — Evaluate cost
 *     For each trajectory, compute a weighted sum of:
 *       - obstacle_cost  (from costmap — penalize proximity to obstacles)
 *       - path_cost      (average distance to the reference global path)
 *       - goal_cost      (distance from trajectory endpoint to goal)
 *       - speed_cost     (penalize slow speeds to encourage progress)
 *
 *   Step 6 — Select best trajectory
 *     Choose the (v, w) pair with the lowest total cost.
 *     If no valid trajectory is found, command zero velocity.
 *
 * @author: user
 * @date: 2026-06-17
 *
 * ********************************************************
 */
#include <pluginlib/class_list_macros.h>

#include "controller/my_dwa_controller.h"

PLUGINLIB_EXPORT_CLASS(rmp::controller::MyDWAController,
                       nav_core::BaseLocalPlanner)

namespace rmp {
namespace controller {

// ═══════════════════════════════════════════════════════════════════
//  Constructor / Destructor
// ═══════════════════════════════════════════════════════════════════

MyDWAController::MyDWAController()
    : initialized_(false), goal_reached_(false), tf_(nullptr),
      costmap_ros_(nullptr) {}

MyDWAController::~MyDWAController() {}

// ═══════════════════════════════════════════════════════════════════
//  initialize()
// ═══════════════════════════════════════════════════════════════════

void MyDWAController::initialize(std::string name, tf2_ros::Buffer *tf,
                                 costmap_2d::Costmap2DROS *costmap_ros) {
  if (initialized_) {
    ROS_WARN("MyDWAController has already been initialized. Doing nothing.");
    return;
  }

  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  ros::NodeHandle nh("~/" + name);

  // Load parameters from the ROS parameter server
  loadParams();

  // Odometry helper
  odom_helper_ =
      std::make_shared<base_local_planner::OdometryHelperRos>("odom");
  std::string odom_topic;
  if (nh.getParam("odom_topic", odom_topic)) {
    odom_helper_->setOdomTopic(odom_topic);
  }

  // Publishers
  local_plan_pub_ = nh.advertise<nav_msgs::Path>("local_plan", 1);
  global_plan_pub_ = nh.advertise<nav_msgs::Path>("global_plan", 1);
  trajectory_marker_pub_ =
      nh.advertise<visualization_msgs::MarkerArray>("explored_trajectories", 1);

  initialized_ = true;

  R_INFO << "MyDWAController initialized! (sim_time=" << params_.sim_time
         << ", vx_samples=" << params_.vx_samples
         << ", vth_samples=" << params_.vtheta_samples << ")";
}

// ═══════════════════════════════════════════════════════════════════
//  loadParams()
// ═══════════════════════════════════════════════════════════════════

void MyDWAController::loadParams() {
  ros::NodeHandle nh("~/" + name_);

  // ── Velocity limits ──
  nh.param("max_vel_x", params_.max_vel_x, 0.55);
  nh.param("min_vel_x", params_.min_vel_x, 0.0);
  nh.param("max_vel_y", params_.max_vel_y, 0.0);
  nh.param("max_vel_theta", params_.max_vel_theta, 1.0);
  nh.param("min_vel_theta", params_.min_vel_theta, -1.0);

  // ── Acceleration limits ──
  nh.param("acc_lim_x", params_.acc_lim_x, 2.5);
  nh.param("acc_lim_y", params_.acc_lim_y, 0.0);
  nh.param("acc_lim_theta", params_.acc_lim_theta, 3.2);

  // ── Simulation ──
  nh.param("sim_time", params_.sim_time, 3.0);
  nh.param("sim_granularity", params_.sim_granularity, 0.025);
  nh.param("control_dt", params_.dt, 0.05);

  // ── Sampling ──
  nh.param("vx_samples", params_.vx_samples, 6.0);
  nh.param("vtheta_samples", params_.vtheta_samples, 20.0);

  // ── Cost weights ──
  nh.param("obstacle_cost_weight", params_.obstacle_cost_weight, 10.0);
  nh.param("path_cost_weight", params_.path_cost_weight, 1.0);
  nh.param("goal_cost_weight", params_.goal_cost_weight, 0.5);
  nh.param("speed_cost_weight", params_.speed_cost_weight, 0.1);

  // ── Obstacle ──
  nh.param("min_obstacle_dist", params_.min_obstacle_dist, 0.5);
  nh.param("max_obstacle_range", params_.max_obstacle_range, 3.0);
  nh.param("robot_radius", params_.robot_radius, 0.15);

  // ── Goal tolerance ──
  nh.param("xy_goal_tolerance", params_.xy_goal_tolerance, 0.10);
  nh.param("yaw_goal_tolerance", params_.yaw_goal_tolerance, 0.10);
}

// ═══════════════════════════════════════════════════════════════════
//  setPlan() / isGoalReached()
// ═══════════════════════════════════════════════════════════════════

bool MyDWAController::setPlan(
    const std::vector<geometry_msgs::PoseStamped> &plan) {
  if (!initialized_) {
    ROS_ERROR("MyDWAController has not been initialized — cannot set plan.");
    return false;
  }
  global_plan_ = plan;
  goal_reached_ = false;
  R_INFO << "MyDWA: Received new global plan with " << plan.size()
         << " waypoints.";
  return true;
}

bool MyDWAController::isGoalReached() {
  if (!initialized_) {
    return false;
  }
  if (!costmap_ros_->getRobotPose(current_pose_)) {
    ROS_ERROR("MyDWA: Could not get robot pose");
    return false;
  }
  return goal_reached_;
}

// ═══════════════════════════════════════════════════════════════════
//  computeVelocityCommands()
//  This is the MAIN entry point called every control cycle.
// ═══════════════════════════════════════════════════════════════════

bool MyDWAController::computeVelocityCommands(geometry_msgs::Twist &cmd_vel) {
  if (!initialized_) {
    ROS_ERROR("MyDWAController has not been initialized!");
    return false;
  }

  // ─── Step 0: Get current robot pose ───────────────────────────
  if (!costmap_ros_->getRobotPose(current_pose_)) {
    ROS_ERROR("MyDWA: Could not get robot pose");
    return false;
  }

  const double px = current_pose_.pose.position.x;
  const double py = current_pose_.pose.position.y;
  const double pth = tf2::getYaw(current_pose_.pose.orientation);

  // ─── Check if the goal is empty ───────────────────────────────
  if (global_plan_.empty()) {
    ROS_WARN("MyDWA: Global plan is empty. Stopping robot.");
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    return true;
  }

  // ─── Check if we have reached the goal ────────────────────────
  const geometry_msgs::PoseStamped &goal_pose = global_plan_.back();
  const double dx_goal = goal_pose.pose.position.x - px;
  const double dy_goal = goal_pose.pose.position.y - py;
  const double dist_to_goal = std::hypot(dx_goal, dy_goal);
  const double angle_to_goal = angles::shortest_angular_distance(
      pth, tf2::getYaw(goal_pose.pose.orientation));

  if (dist_to_goal < params_.xy_goal_tolerance &&
      std::fabs(angle_to_goal) < params_.yaw_goal_tolerance) {
    goal_reached_ = true;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    ROS_INFO("MyDWA: Goal reached!");
    publishLocalPlan({}); // publish empty plan
    return true;
  }

  // ─── Step 1: Get current velocity from odometry ───────────────
  geometry_msgs::PoseStamped robot_vel_msg;
  odom_helper_->getRobotVel(robot_vel_msg);

  const double vx = robot_vel_msg.pose.position.x; // linear x velocity
  const double vy = robot_vel_msg.pose.position.y; // linear y velocity
  const double vth =
      tf2::getYaw(robot_vel_msg.pose.orientation); // angular velocity

  // ─── Step 2: Compute the dynamic window ───────────────────────
  double min_v, max_v, min_w, max_w;
  calcDynamicWindow(vx, vy, vth, min_v, max_v, min_w, max_w);

  ROS_DEBUG("MyDWA: vel=(%.3f, %.3f, %.3f)  dyn_window: v=[%.3f, %.3f]  "
            "w=[%.3f, %.3f]",
            vx, vy, vth, min_v, max_v, min_w, max_w);

  // If the dynamic window collapsed to nothing, widen it minimally
  if (max_v - min_v < 1e-6 && max_w - min_w < 1e-6) {
    ROS_DEBUG("MyDWA: dynamic window collapsed, using full velocity limits.");
    min_v = params_.min_vel_x;
    max_v = params_.max_vel_x;
    min_w = params_.min_vel_theta;
    max_w = params_.max_vel_theta;
  }

  // ─── Step 3: Sample velocities within the window ──────────────
  auto velocity_samples = sampleVelocities(min_v, max_v, min_w, max_w);

  if (velocity_samples.empty()) {
    ROS_WARN("MyDWA: No velocities in dynamic window. Stopping.");
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    return false;
  }

  ROS_DEBUG("MyDWA: sampling %zu velocity pairs", velocity_samples.size());

  // ─── Step 4 & 5: For each sample, predict trajectory + evaluate cost ──
  TrajectoryPoint current_pose_pt{px, py, pth};
  std::vector<Trajectory> all_trajectories;
  Trajectory best_traj;
  int valid_count = 0;

  for (const auto &sample : velocity_samples) {
    double sample_v = sample.first;
    double sample_w = sample.second;

    // Step 4: Predict trajectory
    Trajectory traj = predictTrajectory(current_pose_pt, sample_v, sample_w);
    traj.v = sample_v;
    traj.w = sample_w;

    // Step 5: Evaluate cost
    traj.cost = evaluateCost(traj);

    if (traj.cost < std::numeric_limits<double>::max()) {
      valid_count++;
    }

    all_trajectories.push_back(traj);

    // Keep the lowest-cost trajectory
    if (traj.cost < best_traj.cost) {
      best_traj = traj;
    }
  }

  if (valid_count > 0) {
    ROS_INFO("MyDWA: %d/%zu valid, best cost=%.3f (v=%.3f, w=%.3f)",
             valid_count, velocity_samples.size(), best_traj.cost, best_traj.v,
             best_traj.w);
  }

  // ─── Step 6: Output the best velocity ─────────────────────────
  if (best_traj.cost < std::numeric_limits<double>::max()) {
    cmd_vel.linear.x = best_traj.v;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = best_traj.w;

    // Publish the local plan (best trajectory path)
    publishLocalPlan(best_traj.pts);
  } else {
    // No valid trajectory found — stop the robot
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    ROS_WARN("MyDWA: No valid trajectory found out of %zu samples. Stopping.",
             velocity_samples.size());
    publishLocalPlan({});
    return false; // Signal failure to move_base so it knows to retry
  }

  // Publish visualization
  publishTrajectoryMarkers(all_trajectories);
  publishGlobalPlan();

  return true;
}

// ═══════════════════════════════════════════════════════════════════
//  calcDynamicWindow()
//  Computes the set of velocities reachable in the next control cycle.
// ═══════════════════════════════════════════════════════════════════

void MyDWAController::calcDynamicWindow(double vx, double vy, double vth,
                                        double &min_v, double &max_v,
                                        double &min_w, double &max_w) const {
  // Constraint (a): velocity limits from configuration
  double v_max_lim = params_.max_vel_x;
  double v_min_lim = params_.min_vel_x;
  double w_max_lim = params_.max_vel_theta;
  double w_min_lim = params_.min_vel_theta;

  // Constraint (b): reachable with current acceleration
  double v_max_acc = vx + params_.acc_lim_x * params_.dt;
  double v_min_acc = vx - params_.acc_lim_x * params_.dt;
  double w_max_acc = vth + params_.acc_lim_theta * params_.dt;
  double w_min_acc = vth - params_.acc_lim_theta * params_.dt;

  // Intersection: take the tightest bounds (most restrictive)
  min_v = std::max(v_min_lim, v_min_acc);
  max_v = std::min(v_max_lim, v_max_acc);
  min_w = std::max(w_min_lim, w_min_acc);
  max_w = std::min(w_max_lim, w_max_acc);

  // Ensure the window is valid
  if (min_v > max_v)
    std::swap(min_v, max_v);
  if (min_w > max_w)
    std::swap(min_w, max_w);
}

// ═══════════════════════════════════════════════════════════════════
//  sampleVelocities()
//  Discretize the dynamic window into a grid of (v, w) samples.
// ═══════════════════════════════════════════════════════════════════

std::vector<std::pair<double, double>>
MyDWAController::sampleVelocities(double min_v, double max_v, double min_w,
                                  double max_w) const {
  std::vector<std::pair<double, double>> samples;

  // Number of samples
  int n_v = static_cast<int>(params_.vx_samples);
  int n_w = static_cast<int>(params_.vtheta_samples);

  // Handle edge cases where the window is a single point
  if (n_v < 1)
    n_v = 1;
  if (n_w < 1)
    n_w = 1;

  double step_v = (max_v - min_v) / std::max(n_v - 1, 1);
  double step_w = (max_w - min_w) / std::max(n_w - 1, 1);

  for (int i = 0; i < n_v; ++i) {
    double v = min_v + i * step_v;
    for (int j = 0; j < n_w; ++j) {
      double w = min_w + j * step_w;
      samples.emplace_back(v, w);
    }
  }

  return samples;
}

// ═══════════════════════════════════════════════════════════════════
//  predictTrajectory()
//  Given a constant velocity (v, w), simulate forward in time using
//  the differential-drive / unicycle motion model:
//
//    x' = x + v * cos(theta) * dt
//    y' = y + v * sin(theta) * dt
//    theta' = theta + w * dt
//
// ═══════════════════════════════════════════════════════════════════

Trajectory MyDWAController::predictTrajectory(const TrajectoryPoint &pos,
                                              double v, double w) const {
  Trajectory traj;
  traj.v = v;
  traj.w = w;

  double x = pos.x;
  double y = pos.y;
  double th = pos.theta;

  // Use a fixed time step (max 0.1s) for consistent simulation regardless of
  // velocity. This avoids the problem where small v produces huge dt =
  // granularity/|v|.
  double sim_time = params_.sim_time;
  double dt = std::min(params_.dt, 0.1);
  int steps = static_cast<int>(sim_time / dt);
  if (steps < 1)
    steps = 1;
  dt = sim_time / steps;

  // Add the starting pose
  traj.pts.emplace_back(x, y, th);

  for (int i = 0; i < steps; ++i) {
    // Unicycle / differential-drive motion model
    x += v * std::cos(th) * dt;
    y += v * std::sin(th) * dt;
    th += w * dt;

    traj.pts.emplace_back(x, y, th);
  }

  return traj;
}

// ═══════════════════════════════════════════════════════════════════
//  evaluateCost()
//  Weighted sum of individual cost terms.
//  Returns std::numeric_limits<double>::max() if the trajectory is
//  invalid (e.g., hits an obstacle).
// ═══════════════════════════════════════════════════════════════════

double MyDWAController::evaluateCost(const Trajectory &traj) const {
  // Obstacle cost — if the trajectory goes through a lethal obstacle,
  // mark it as invalid (cost = max)
  double occ_cost = calcObstacleCost(traj);
  if (occ_cost < 0.0) {
    return std::numeric_limits<double>::max();
  }

  // Path alignment cost
  double path_cost = calcPathCost(traj);

  // Goal distance cost
  double goal_cost = calcGoalCost(traj);

  // Speed cost (prefer higher speeds for forward progress)
  double speed_cost = calcSpeedCost(traj.v);

  // Weighted sum
  double total = params_.obstacle_cost_weight * occ_cost +
                 params_.path_cost_weight * path_cost +
                 params_.goal_cost_weight * goal_cost +
                 params_.speed_cost_weight * speed_cost;

  return total;
}

// ═══════════════════════════════════════════════════════════════════
//  calcObstacleCost()
//  For each point on the trajectory, look up the costmap cost. If any
//  point is at or beyond LETHAL_OBSTACLE, return -1 (invalid).
//  Otherwise, return the sum of (1 / distance_to_nearest_obstacle)
//  as a proximity penalty.
// ═══════════════════════════════════════════════════════════════════

double MyDWAController::calcObstacleCost(const Trajectory &traj) const {
  // ── Strategy ─────────────────────────────────────────────────────────
  // 1. Interpolate along each trajectory segment (so no small obstacle is
  // missed)
  // 2. For each interpolated point, check not just the center but also
  // left/right
  //    offsets by robot_radius, to account for the robot's physical width.
  // 3. If ANY checked cell has cost >= INSCRIBED_INFLATED_OBSTACLE, the robot
  //    body would touch the obstacle → trajectory is INVALID (return -1).
  // 4. Otherwise, compute the penalty as the MAXIMUM normalized cost seen
  //    across all checked cells.  A single close brush with an obstacle makes
  //    the whole trajectory expensive (exponential penalty).

  const double inscribed = costmap_2d::INSCRIBED_INFLATED_OBSTACLE; // ~253
  const double lethal = costmap_2d::LETHAL_OBSTACLE;                // 254
  double r = params_.robot_radius;
  double max_norm = 0.0; // highest normalized cost seen

  // Lateral offset directions (robot body)
  // For a diff-drive robot, left/right relative to heading matters
  // We approximate by checking points offset perpendicular to the trajectory
  std::vector<std::pair<double, double>> offsets = {
      {0.0, 0.0}, // center
      {r, 0.0},   // right
      {-r, 0.0}   // left
  };

  for (size_t i = 0; i + 1 < traj.pts.size(); ++i) {
    const auto &p1 = traj.pts[i];
    const auto &p2 = traj.pts[i + 1];

    // Segment direction (heading)
    double seg_th = std::atan2(p2.y - p1.y, p2.x - p1.x);
    double seg_len = std::hypot(p2.x - p1.x, p2.y - p1.y);

    int steps_in_seg =
        std::max(1, static_cast<int>(seg_len / params_.sim_granularity));

    for (int s = 0; s <= steps_in_seg; ++s) {
      double frac =
          (steps_in_seg > 0) ? static_cast<double>(s) / steps_in_seg : 0.0;
      double cx = p1.x + (p2.x - p1.x) * frac;
      double cy = p1.y + (p2.y - p1.y) * frac;

      // Skip the very first point (current robot pose)
      if (i == 0 && s == 0)
        continue;

      // Check center + left/right offsets to account for robot width
      for (const auto &off : offsets) {
        // Rotate offset by segment heading
        double px = cx + off.first * std::cos(seg_th + M_PI / 2.0) -
                    off.second * std::sin(seg_th + M_PI / 2.0);
        double py = cy + off.first * std::sin(seg_th + M_PI / 2.0) +
                    off.second * std::cos(seg_th + M_PI / 2.0);

        unsigned char cost = getCostmapCost(px, py);

        // Robot body would touch obstacle → invalid
        if (cost >= inscribed) {
          return -1.0;
        }

        // Track the worst cell encountered
        if (cost > 0) {
          double norm = static_cast<double>(cost) / lethal; // [0, 0.996]
          if (norm > max_norm)
            max_norm = norm;
        }
      }
    }
  }

  if (max_norm <= 0.0)
    return 0.0;

  // Exponential scaling: cost grows very fast as norm → 1.0
  // At norm=0.5 → cost=0.25, at norm=0.9 → cost=4.3, at norm=0.99 → cost=52
  double penalty = max_norm * max_norm / (1.0 - max_norm + 1e-6);

  return penalty;
}

// ═══════════════════════════════════════════════════════════════════
//  calcPathCost()
//  Average Euclidean distance from each trajectory point to the
//  nearest point on the global reference path. Lower is better.
// ═══════════════════════════════════════════════════════════════════

double MyDWAController::calcPathCost(const Trajectory &traj) const {
  if (global_plan_.empty() || traj.pts.empty()) {
    return 0.0;
  }

  double total_dist = 0.0;
  for (const auto &pt : traj.pts) {
    total_dist += distanceToPath(pt.x, pt.y);
  }

  return total_dist / traj.pts.size();
}

// ═══════════════════════════════════════════════════════════════════
//  calcGoalCost()
//  Euclidean distance from the END of the trajectory to the final goal.
//  Lower is better (trajectory ends closer to goal).
// ═══════════════════════════════════════════════════════════════════

double MyDWAController::calcGoalCost(const Trajectory &traj) const {
  if (global_plan_.empty() || traj.pts.empty()) {
    return 0.0;
  }

  const auto &goal = global_plan_.back();
  const auto &endpoint = traj.pts.back();

  double dx = endpoint.x - goal.pose.position.x;
  double dy = endpoint.y - goal.pose.position.y;

  return std::hypot(dx, dy);
}

// ═══════════════════════════════════════════════════════════════════
//  calcSpeedCost()
//  Prefer higher translational speeds. Returns a cost that decreases
//  as speed increases:
//    cost = max_speed - |v|
//  So v = max_speed → cost = 0 (best), v = 0 → cost = max_speed (worst).
// ═══════════════════════════════════════════════════════════════════

double MyDWAController::calcSpeedCost(double v) const {
  double max_speed = std::max(std::fabs(params_.max_vel_x), 1e-3);
  return max_speed - std::fabs(v);
}

// ═══════════════════════════════════════════════════════════════════
//  getCostmapCost()
// ═══════════════════════════════════════════════════════════════════

unsigned char MyDWAController::getCostmapCost(double wx, double wy) const {
  if (!costmap_ros_) {
    return costmap_2d::LETHAL_OBSTACLE;
  }

  costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();
  if (!costmap) {
    return costmap_2d::LETHAL_OBSTACLE;
  }

  unsigned int mx, my;
  if (!costmap->worldToMap(wx, wy, mx, my)) {
    // Out of bounds — treat as lethal
    return costmap_2d::LETHAL_OBSTACLE;
  }

  return costmap->getCost(mx, my);
}

// ═══════════════════════════════════════════════════════════════════
//  distanceToPath()
//  Brute-force minimum distance from (x, y) to the global plan.
// ═══════════════════════════════════════════════════════════════════

double MyDWAController::distanceToPath(double x, double y) const {
  if (global_plan_.empty()) {
    return 0.0;
  }

  double min_dist = std::numeric_limits<double>::max();
  for (const auto &pose : global_plan_) {
    double dx = x - pose.pose.position.x;
    double dy = y - pose.pose.position.y;
    double dist = std::hypot(dx, dy);
    if (dist < min_dist) {
      min_dist = dist;
    }
  }
  return min_dist;
}

// ═══════════════════════════════════════════════════════════════════
//  publishLocalPlan()
//  Publish the best trajectory as a nav_msgs/Path for visualization.
// ═══════════════════════════════════════════════════════════════════

void MyDWAController::publishLocalPlan(
    const std::vector<TrajectoryPoint> &traj_pts) {
  nav_msgs::Path path_msg;
  path_msg.header.frame_id = costmap_ros_->getGlobalFrameID();
  path_msg.header.stamp = ros::Time::now();

  for (const auto &pt : traj_pts) {
    geometry_msgs::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position.x = pt.x;
    pose.pose.position.y = pt.y;
    pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, pt.theta);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    path_msg.poses.push_back(pose);
  }

  local_plan_pub_.publish(path_msg);
}

// ═══════════════════════════════════════════════════════════════════
//  publishTrajectoryMarkers()
//  Publish all explored trajectories as visualization markers (rviz).
//  Green = valid (cost >= 0), Red = invalid (cost < 0).
// ═══════════════════════════════════════════════════════════════════

void MyDWAController::publishTrajectoryMarkers(
    const std::vector<Trajectory> &trajectories) {
  visualization_msgs::MarkerArray marker_array;
  int id = 0;

  for (const auto &traj : trajectories) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = costmap_ros_->getGlobalFrameID();
    marker.header.stamp = ros::Time::now();
    marker.ns = "explored_trajectories";
    marker.id = id++;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;

    // Color: green for valid, red for invalid
    bool valid = (traj.cost < std::numeric_limits<double>::max());
    marker.color.r = valid ? 0.0 : 1.0;
    marker.color.g = valid ? 1.0 : 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.5;

    marker.scale.x = 0.02; // line width

    for (const auto &pt : traj.pts) {
      geometry_msgs::Point p;
      p.x = pt.x;
      p.y = pt.y;
      p.z = 0.0;
      marker.points.push_back(p);
    }

    marker_array.markers.push_back(marker);
  }

  trajectory_marker_pub_.publish(marker_array);
}

// ═══════════════════════════════════════════════════════════════════
//  publishGlobalPlan()
// ═══════════════════════════════════════════════════════════════════

void MyDWAController::publishGlobalPlan() {
  nav_msgs::Path path_msg;
  path_msg.header.frame_id = costmap_ros_->getGlobalFrameID();
  path_msg.header.stamp = ros::Time::now();
  path_msg.poses = global_plan_;
  global_plan_pub_.publish(path_msg);
}

} // namespace controller
} // namespace rmp
