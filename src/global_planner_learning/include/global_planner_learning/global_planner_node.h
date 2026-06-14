/**
 * @file global_planner_node.h
 * @brief 独立全局路径规划节点（不依赖 move_base）
 * @author learner
 * @version 0.1
 *
 * 功能：
 * 1. 通过 map_server 加载地图
 * 2. 接收 RViz 2D Nav Goal 作为目标点
 * 3. 接收 RViz 2D Pose Estimate 或 TF 作为起点
 * 4. 使用 A* 算法规划路径
 * 5. 可视化路径和搜索过程
 */

#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include "global_planner_learning/astar_planner.h"
#include "global_planner_learning/dijkstra_planner.h"
#include "global_planner_learning/rrt_planner.h"
#include "global_planner_learning/planner_base.h"

namespace global_planner_learning {

class GlobalPlannerNode {
public:
  GlobalPlannerNode();
  ~GlobalPlannerNode();

  /**
   * @brief 主循环（处理等待地图加载等）
   */
  void run();

private:
  /**
   * @brief 地图回调函数
   */
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);

  /**
   * @brief RViz 2D Nav Goal 回调（终点）
   */
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

  /**
   * @brief RViz 2D Pose Estimate 回调（起点）
   */
  void
  startCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

  /**
   * @brief 执行路径规划
   */
  void doPlan();

  /**
   * @brief 发布路径可视化
   */
  void publishPlan(const std::vector<geometry_msgs::PoseStamped> &plan);

  /**
   * @brief 发布搜索过程可视化（已展开节点）
   */
  void publishVisitedNodes(const std::vector<Node> &nodes);

  /**
   * @brief 从 TF 获取机器人当前位置
   * @return true 成功获取
   */
  bool getRobotPoseFromTF(geometry_msgs::PoseStamped &pose);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // 订阅
  ros::Subscriber map_sub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber start_sub_;

  // 发布
  ros::Publisher plan_pub_;
  ros::Publisher visited_pub_;
  ros::Publisher stats_pub_; // 规划统计信息（std_msgs/String）

  // TF 监听
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // 数据
  nav_msgs::OccupancyGrid::ConstPtr map_;
  geometry_msgs::PoseStamped start_pose_;
  geometry_msgs::PoseStamped goal_pose_;
  bool has_start_;
  bool has_goal_;
  bool map_ready_;

  // 规划器（基类指针，支持多态切换）
  PathPlannerBase *planner_;

  // 参数
  std::string map_frame_;    // 地图坐标系
  std::string planner_name_; // 使用的规划器名称
  bool allow_unknown_;       // 是否允许穿越未知区域
  bool use_8_connectivity_;  // 是否使用 8 连通
  bool use_tf_for_start_; // 是否从 TF 获取起点（替代 RViz 2D Pose）
};

} // namespace global_planner_learning
