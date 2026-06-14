/**
 * @file global_planner_node.cpp
 * @brief 独立全局路径规划节点实现
 * @author learner
 * @version 0.1
 */

#include <std_msgs/String.h>

#include "global_planner_learning/global_planner_node.h"

namespace global_planner_learning {

// ==================== 构造函数 ====================
GlobalPlannerNode::GlobalPlannerNode()
    : private_nh_("~"), tf_listener_(tf_buffer_), has_start_(false),
      has_goal_(false), map_ready_(false), planner_(nullptr) {
  // ---- 读取参数 ----
  private_nh_.param("map_frame", map_frame_, std::string("map"));
  private_nh_.param("planner_name", planner_name_, std::string("AStar"));
  private_nh_.param("allow_unknown", allow_unknown_, false);
  private_nh_.param("use_8_connectivity", use_8_connectivity_, true);
  // 如果为 true，则从 TF (map→base_footprint) 获取起点，否则从 /initialpose
  // 获取
  private_nh_.param("use_tf_for_start", use_tf_for_start_, true);

  // ---- 订阅 ----
  // 地图（由 map_server 发布）
  map_sub_ = nh_.subscribe("/map", 1, &GlobalPlannerNode::mapCallback, this);

  // RViz 2D Nav Goal → 目标点
  goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1,
                            &GlobalPlannerNode::goalCallback, this);

  // RViz 2D Pose Estimate → 起点（当 use_tf_for_start=false 时使用）
  start_sub_ =
      nh_.subscribe("/initialpose", 1, &GlobalPlannerNode::startCallback, this);

  // ---- 发布 ----
  // 规划得到的全局路径
  plan_pub_ = nh_.advertise<nav_msgs::Path>("/plan", 1, true);

  // 搜索过程中访问的节点（可视化为 OccupancyGrid）
  visited_pub_ =
      nh_.advertise<nav_msgs::OccupancyGrid>("/visited_nodes", 1, true);

  // 规划统计信息（字符串，便于 rviz 或命令行查看）
  stats_pub_ = nh_.advertise<std_msgs::String>("/plan_stats", 1, true);

  ROS_INFO("[GlobalPlanner] 节点已启动，等待地图...");
}

// ==================== 析构函数 ====================
GlobalPlannerNode::~GlobalPlannerNode() {
  delete planner_;
  planner_ = nullptr;
}

// ==================== 主循环 ====================
void GlobalPlannerNode::run() {
  ros::Rate rate(10);
  while (ros::ok()) {
    ros::spinOnce();

    // 如果地图已就绪且 use_tf_for_start=true，自动获取起点
    if (map_ready_ && use_tf_for_start_) {
      geometry_msgs::PoseStamped robot_pose;
      if (getRobotPoseFromTF(robot_pose)) {
        if (!has_start_ ||
            std::abs(robot_pose.pose.position.x - start_pose_.pose.position.x) >
                0.1 ||
            std::abs(robot_pose.pose.position.y - start_pose_.pose.position.y) >
                0.1) {
          start_pose_ = robot_pose;
          has_start_ = true;
          ROS_DEBUG("[GlobalPlanner] 从 TF 更新机器人位置: (%.2f, %.2f)",
                    robot_pose.pose.position.x, robot_pose.pose.position.y);
        }
      }
    }

    // 如果起点和终点都已就绪，执行规划
    if (map_ready_ && has_start_ && has_goal_) {
      doPlan();
      has_goal_ = false; // 规划完成后重置，等待新的目标
    }

    rate.sleep();
  }
}

// ==================== 地图回调 ====================
void GlobalPlannerNode::mapCallback(
    const nav_msgs::OccupancyGrid::ConstPtr &msg) {
  map_ = msg;
  map_frame_ = msg->header.frame_id;
  if (!map_ready_) {
    map_ready_ = true;
    ROS_INFO("[GlobalPlanner] map loaded: %d x %d, resolution %.3f m/px",
             msg->info.width, msg->info.height, msg->info.resolution);

    // ---- 地图就绪后创建规划器 ----
    // 通过基类指针指向具体算法实现，后续可扩展为根据 planner_name_ 参数切换
    delete planner_;
    if (planner_name_ == "AStar") {
      planner_ = new AStarPlanner(map_, allow_unknown_, use_8_connectivity_);
      ROS_INFO("[GlobalPlanner] planner created: %s", planner_name_.c_str());
    } else if (planner_name_ == "Dijkstra") {
      planner_ = new DijkstraPlanner(map_, allow_unknown_, use_8_connectivity_);
      ROS_INFO("[GlobalPlanner] planner created: %s", planner_name_.c_str());
    } else if (planner_name_ == "RRT") {
      // RRT 使用额外的参数：goal_bias, step_size, max_iter
      double goal_bias = 0.1;
      double step_size = 0.5;
      int max_iter = 5000;
      private_nh_.param("rrt_goal_bias", goal_bias, 0.1);
      private_nh_.param("rrt_step_size", step_size, 0.5);
      private_nh_.param("rrt_max_iter", max_iter, 5000);
      planner_ = new RRTPlanner(map_, allow_unknown_, goal_bias, step_size,
                                max_iter);
      ROS_INFO("[GlobalPlanner] planner created: %s (bias=%.2f, step=%.2f, "
               "max_iter=%d)",
               planner_name_.c_str(), goal_bias, step_size, max_iter);
    } else {
      ROS_WARN("[GlobalPlanner] unknown planner '%s', fallback to AStar",
               planner_name_.c_str());
      planner_ = new AStarPlanner(map_, allow_unknown_, use_8_connectivity_);
    }
  }
}

// ==================== 目标点回调 (RViz 2D Nav Goal) ====================
void GlobalPlannerNode::goalCallback(
    const geometry_msgs::PoseStamped::ConstPtr &msg) {
  goal_pose_ = *msg;
  has_goal_ = true;
  ROS_INFO("[GlobalPlanner] received goal pose: (%.2f, %.2f)",
           goal_pose_.pose.position.x, goal_pose_.pose.position.y);
}

// ==================== 起点回调 (RViz 2D Pose Estimate) ====================
void GlobalPlannerNode::startCallback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
  start_pose_.header = msg->header;
  start_pose_.pose = msg->pose.pose;
  has_start_ = true;
  ROS_INFO("[GlobalPlanner] received start pose: (%.2f, %.2f)",
           start_pose_.pose.position.x, start_pose_.pose.position.y);
}

// ==================== 从 TF 获取机器人位置 ====================
bool GlobalPlannerNode::getRobotPoseFromTF(geometry_msgs::PoseStamped &pose) {
  try {
    // 获取当前时间附近的变换
    tf_buffer_.canTransform(map_frame_, "base_footprint", ros::Time(0),
                            ros::Duration(0.5));
    geometry_msgs::TransformStamped transform =
        tf_buffer_.lookupTransform(map_frame_, "base_footprint", ros::Time(0));

    pose.header = transform.header;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;
    return true;
  } catch (tf2::TransformException &ex) {
    ROS_DEBUG_THROTTLE(1.0, "[GlobalPlanner] TF 查询失败: %s", ex.what());
    return false;
  }
}

// ==================== 执行规划 ====================
void GlobalPlannerNode::doPlan() {
  if (!map_ready_ || !has_start_ || !has_goal_ || !planner_)
    return;

  ROS_INFO("[GlobalPlanner] planning... from (%.2f, %.2f) to (%.2f, %.2f)",
           start_pose_.pose.position.x, start_pose_.pose.position.y,
           goal_pose_.pose.position.x, goal_pose_.pose.position.y);

  // 通过基类指针调用规划器（多态）
  std::vector<geometry_msgs::PoseStamped> plan;
  bool success = planner_->makePlan(start_pose_, goal_pose_, plan);

  // 获取统计信息
  PlannerStatistics stats = planner_->getStatistics();

  // 发布路径
  publishPlan(plan);

  // 发布搜索过程可视化
  publishVisitedNodes(planner_->getVisitedNodes());

  // 发布统计信息（String 话题）
  std_msgs::String stats_msg;
  stats_msg.data = stats.toString();
  stats_pub_.publish(stats_msg);

  // 终端输出统计
  ROS_INFO("[GlobalPlanner] %s", stats.toString().c_str());
}

// ==================== 发布路径 ====================
void GlobalPlannerNode::publishPlan(
    const std::vector<geometry_msgs::PoseStamped> &plan) {
  nav_msgs::Path gui_path;
  gui_path.header.frame_id = map_frame_;
  gui_path.header.stamp = ros::Time::now();
  gui_path.poses = plan;
  plan_pub_.publish(gui_path);
}

// ==================== 发布搜索过程可视化 ====================
void GlobalPlannerNode::publishVisitedNodes(const std::vector<Node> &nodes) {
  if (nodes.empty() || !map_)
    return;

  // 创建一个 OccupancyGrid 来显示搜索过程
  // 白色=未搜索，灰色=已访问，黑色=障碍物
  nav_msgs::OccupancyGrid grid;
  grid.header.frame_id = map_frame_;
  grid.header.stamp = ros::Time::now();
  grid.info = map_->info;
  grid.data = map_->data; // 从原始地图开始

  // 将访问过的节点标记为灰色（50 表示已搜索区域）
  for (const auto &node : nodes) {
    int idx = node.y * grid.info.width + node.x;
    if (grid.data[idx] < 50) {
      grid.data[idx] = 50;
    }
  }

  visited_pub_.publish(grid);
}

} // namespace global_planner_learning

// ==================== 主函数 ====================
int main(int argc, char **argv) {
  ros::init(argc, argv, "global_planner_node");

  global_planner_learning::GlobalPlannerNode node;
  node.run();

  return 0;
}
