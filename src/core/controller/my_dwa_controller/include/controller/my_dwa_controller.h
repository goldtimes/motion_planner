/**
 * *********************************************************
 *
 * @file: my_dwa_controller.h
 * @brief: A clean from-scratch implementation of the Dynamic Window Approach
 * (DWA) for local robot navigation. This implementation does NOT depend on the
 *         base_local_planner's internal cost functions — it implements its own
 *         velocity sampling, trajectory prediction, and cost evaluation.
 *
 * DWA Algorithm Summary:
 *   1. Compute the "dynamic window" — the set of (v, w) reachable in the next
 *      control cycle given current velocity and acceleration limits.
 *   2. Sample velocities within the dynamic window at a given resolution.
 *   3. For each sampled (v, w), simulate forward in time to generate a
 * trajectory.
 *   4. Evaluate each trajectory with a cost function that considers:
 *      - Obstacle clearance (costmap)
 *      - Path alignment (distance to reference path)
 *      - Goal distance
 *      - Forward progress (prefer higher speeds)
 *   5. Select the velocity pair with the lowest cost.
 *
 * @author: user
 * @date: 2026-06-17
 *
 * ********************************************************
 */
#ifndef RMP_CONTROLLER_MY_DWA_CONTROLLER_H_
#define RMP_CONTROLLER_MY_DWA_CONTROLLER_H_

#include <base_local_planner/odometry_helper_ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf2/utils.h>
#include <tf2_ros/buffer.h>

#include <angles/angles.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "common/util/log.h"

namespace rmp {
namespace controller {

/**
 * @brief A single 2D pose along a trajectory: (x, y, theta)
 */
struct TrajectoryPoint {
  double x;
  double y;
  double theta;
  TrajectoryPoint() : x(0.0), y(0.0), theta(0.0) {}
  TrajectoryPoint(double x, double y, double theta)
      : x(x), y(y), theta(theta) {}
};

/**
 * @brief A trajectory generated from a single velocity sample (v, w),
 *        consisting of a sequence of poses and an associated cost.
 */
struct Trajectory {
  double v;                         ///< linear velocity of this trajectory
  double w;                         ///< angular velocity of this trajectory
  double cost;                      ///< total cost (lower is better)
  std::vector<TrajectoryPoint> pts; ///< predicted poses

  Trajectory() : v(0.0), w(0.0), cost(std::numeric_limits<double>::max()) {}
};

/**
 * @class MyDWAController
 * @brief A nav_core::BaseLocalPlanner plugin implementing the Dynamic Window
 * Approach from scratch.
 */
class MyDWAController : public nav_core::BaseLocalPlanner {
public:
  MyDWAController();
  ~MyDWAController() override;

  /**
   * @brief Initialize the planner
   */
  void initialize(std::string name, tf2_ros::Buffer *tf,
                  costmap_2d::Costmap2DROS *costmap_ros) override;

  /**
   * @brief Set a new global plan
   */
  bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan) override;

  /**
   * @brief Check if the goal has been reached
   */
  bool isGoalReached() override;

  /**
   * @brief Compute velocity commands for the robot base
   * @param cmd_vel Output: velocity command to send to the base
   * @return true if a valid trajectory was found
   */
  bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel) override;

private:
  // ── Configuration parameters (loaded from ROS param server) ──────────
  struct Params {
    // ── Velocity limits ──
    double max_vel_x = 0.55;     ///< max forward linear velocity [m/s]
    double min_vel_x = -0.1;     ///< min forward linear velocity [m/s]
    double max_vel_y = 0.0;      ///< max lateral velocity (for diff drive = 0)
    double max_vel_theta = 1.0;  ///< max angular velocity [rad/s]
    double min_vel_theta = -1.0; ///< min angular velocity [rad/s]

    // ── Acceleration limits ──
    double acc_lim_x = 2.5;     ///< max x acceleration [m/s^2]
    double acc_lim_y = 0.0;     ///< max y acceleration
    double acc_lim_theta = 3.2; ///< max angular acceleration [rad/s^2]

    // ── Simulation parameters ──
    double sim_time = 3.0;          ///< how far into the future to simulate [s]
    double sim_granularity = 0.025; ///< step size along trajectory [m]
    double dt = 0.05;               ///< time step for trajectory simulation [s]

    // ── Sampling resolution ──
    double vx_samples = 6;      ///< number of linear velocity samples
    double vtheta_samples = 20; ///< number of angular velocity samples

    // ── Cost weights ──
    double obstacle_cost_weight = 10.0; ///< weight for obstacle cost
    double path_cost_weight = 1.0;      ///< weight for path alignment cost
    double goal_cost_weight = 0.5;      ///< weight for goal distance cost
    double speed_cost_weight = 0.1;     ///< weight for encouraging higher speed

    // ── Obstacle avoidance ──
    double min_obstacle_dist = 0.5;  ///< minimum safe distance to obstacle [m]
    double max_obstacle_range = 3.0; ///< max range to consider obstacles [m]
    double robot_radius =
        0.15; ///< robot body radius for footprint checking [m]

    // ── Goal tolerance ──
    double xy_goal_tolerance =
        0.10; ///< position tolerance to consider goal reached [m]
    double yaw_goal_tolerance = 0.10; ///< orientation tolerance [rad]
  };

  Params params_;

  // ── State ──
  bool initialized_ = false;
  bool goal_reached_ = false;

  std::string name_;
  tf2_ros::Buffer *tf_ = nullptr;
  costmap_2d::Costmap2DROS *costmap_ros_ = nullptr;
  std::shared_ptr<base_local_planner::OdometryHelperRos> odom_helper_;

  geometry_msgs::PoseStamped current_pose_;
  std::vector<geometry_msgs::PoseStamped> global_plan_;

  // ── Publishers (for visualization) ──
  ros::Publisher local_plan_pub_;
  ros::Publisher trajectory_marker_pub_;
  ros::Publisher global_plan_pub_;

  // ── Core DWA Algorithm ──────────────────────────────────────────────

  /**
   * @brief Compute the dynamic window: velocities reachable in the next control
   *        cycle given current velocity and acceleration limits.
   * @param vx   current linear x velocity
   * @param vy   current linear y velocity
   * @param vth  current angular velocity
   * @param[out] min_v  minimum linear velocity in the window
   * @param[out] max_v  maximum linear velocity in the window
   * @param[out] min_w  minimum angular velocity in the window
   * @param[out] max_w  maximum angular velocity in the window
   */
  void calcDynamicWindow(double vx, double vy, double vth, double &min_v,
                         double &max_v, double &min_w, double &max_w) const;

  /**
   * @brief Sample velocity pairs (v, w) within the dynamic window.
   * @param min_v  min linear velocity
   * @param max_v  max linear velocity
   * @param min_w  min angular velocity
   * @param max_w  max angular velocity
   * @return list of (v, w) pairs to evaluate
   */
  std::vector<std::pair<double, double>> sampleVelocities(double min_v,
                                                          double max_v,
                                                          double min_w,
                                                          double max_w) const;

  /**
   * @brief Simulate a trajectory forward in time given a constant velocity (v,
   * w).
   * @param pos  initial pose [x, y, theta]
   * @param v    linear velocity
   * @param w    angular velocity
   * @return the predicted trajectory
   */
  Trajectory predictTrajectory(const TrajectoryPoint &pos, double v,
                               double w) const;

  /**
   * @brief Evaluate the cost of a trajectory using the costmap and reference
   * path.
   * @param traj  the trajectory to evaluate
   * @return total cost (lower = better). Returns max double if invalid.
   */
  double evaluateCost(const Trajectory &traj) const;

  /**
   * @brief Obstacle cost: check how close the trajectory gets to obstacles
   *        in the costmap.
   * @param traj  trajectory to check
   * @return cost (0 = safe, higher = closer to or in obstacle)
   */
  double calcObstacleCost(const Trajectory &traj) const;

  /**
   * @brief Path alignment cost: average distance from trajectory points
   *        to the nearest point on the reference path.
   * @param traj  trajectory to evaluate
   * @return cost (lower = trajectory follows path better)
   */
  double calcPathCost(const Trajectory &traj) const;

  /**
   * @brief Goal distance cost: Euclidean distance from the end of the
   * trajectory to the final goal.
   * @param traj  trajectory to evaluate
   * @return cost (lower = closer to goal)
   */
  double calcGoalCost(const Trajectory &traj) const;

  /**
   * @brief Speed cost: prefer higher translational speeds for forward progress.
   * @param v  linear velocity of the trajectory
   * @return cost (lower = higher speed, so faster movement is encouraged)
   */
  double calcSpeedCost(double v) const;

  /**
   * @brief Convert (x, y) in world coords to costmap cell, return cost.
   *        Returns costmap_2d::LETHAL_OBSTACLE if out of bounds.
   * @param wx  world x
   * @param wy  world y
   * @return costmap cost at that cell
   */
  unsigned char getCostmapCost(double wx, double wy) const;

  /**
   * @brief Find the minimum Euclidean distance from point (x, y) to the
   *        reference path (global_plan_).
   * @param x  point x
   * @param y  point y
   * @return minimum distance to path
   */
  double distanceToPath(double x, double y) const;

  /**
   * @brief Load parameters from the ROS parameter server
   */
  void loadParams();

  // ── Helpers ──
  void publishLocalPlan(const std::vector<TrajectoryPoint> &traj_pts);
  void publishTrajectoryMarkers(const std::vector<Trajectory> &trajectories);
  void publishGlobalPlan();
};

} // namespace controller
} // namespace rmp

#endif // RMP_CONTROLLER_MY_DWA_CONTROLLER_H_
