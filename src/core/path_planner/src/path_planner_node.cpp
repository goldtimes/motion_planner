#include "path_planner_node.h"
#include "util/log.h"
#include "util/visualizer.h"
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rmp::path_planner::PathPlannerNode,
                       nav_core::BaseGlobalPlanner)
using namespace rmp::path_planner;
using Visualizer = rmp::common::util::Visualizer;

PathPlannerNode::PathPlannerNode() : initialized_(false), g_planner_(nullptr) {}
PathPlannerNode::PathPlannerNode(std::string name,
                                 costmap_2d::Costmap2DROS *costmap_ros)
    : PathPlannerNode() {
  initialize(name, costmap_ros);
}

void PathPlannerNode::initialize(std::string name,
                                 costmap_2d::Costmap2DROS *costmapRos) {
  costmap_ros_ = costmapRos;
  initialize(name);
}
void PathPlannerNode::initialize(std::string name) {
  if (!initialized_) {
    initialized_ = true;

    // initialize ROS node
    ros::NodeHandle private_nh("~/" + name);

    // costmap frame ID
    frame_id_ = costmap_ros_->getGlobalFrameID();

    PathPlannerFactory::PlannerProps path_planner_props;
    if (!PathPlannerFactory::createPlanner(private_nh, costmap_ros_,
                                           path_planner_props)) {
      R_ERROR << "Create path planner failed.";
    }

    g_planner_ = path_planner_props.planner_ptr;
    planner_type_ = path_planner_props.planner_type;

    // register planning publisher
    plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
    points_pub_ =
        private_nh.advertise<visualization_msgs::Marker>("key_points", 1);
    lines_pub_ =
        private_nh.advertise<visualization_msgs::Marker>("safety_corridor", 1);
    tree_pub_ =
        private_nh.advertise<visualization_msgs::Marker>("random_tree", 1);
    particles_pub_ =
        private_nh.advertise<visualization_msgs::Marker>("particles", 1);

    // register explorer visualization publisher
    expand_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("expand", 1);

    // register planning service
    make_plan_srv_ = private_nh.advertiseService(
        "make_plan", &PathPlannerNode::makePlanService, this);
  } else {
    ROS_WARN("This planner has already been initialized, you can't call it "
             "twice, doing "
             "nothing");
  }
}

bool PathPlannerNode::makePlan(const geometry_msgs::PoseStamped &start,
                               const geometry_msgs::PoseStamped &goal,
                               std::vector<geometry_msgs::PoseStamped> &plan) {
  // TODO: add tolerance
  double tolerance = 0.1;
  return makePlan(start, goal, tolerance, plan);
}
bool PathPlannerNode::makePlan(const geometry_msgs::PoseStamped &start,
                               const geometry_msgs::PoseStamped &goal,
                               double tolerance,
                               std::vector<geometry_msgs::PoseStamped> &plan) {}

bool PathPlannerNode::makePlanService(nav_msgs::GetPlan::Request &req,
                                      nav_msgs::GetPlan::Response &resp) {
  makePlan(req.start, req.goal, resp.plan.poses);
  resp.plan.header.stamp = ros::Time::now();
  resp.plan.header.frame_id = frame_id_;

  return true;
}

bool PathPlannerNode::_getPlanFromPath(
    const common::geometry::Points3d &path,
    std::vector<geometry_msgs::PoseStamped> &plan) {
  if (!initialized_) {
    R_ERROR << "This planner has not been initialized yet, but it is being "
               "used, please "
               "call initialize() before use";
    return false;
  }
  plan.clear();

  for (const auto &pt : path) {
    // coding as message type
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = frame_id_;
    pose.pose.position.x = pt.x();
    pose.pose.position.y = pt.y();
    pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, pt.theta());
    pose.pose.orientation.x = q.getX();
    pose.pose.orientation.y = q.getY();
    pose.pose.orientation.z = q.getZ();
    pose.pose.orientation.w = q.getW();
    plan.push_back(pose);
  }

  return !plan.empty();
}
