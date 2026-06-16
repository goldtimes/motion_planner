#pragma once

#include "path_planner.h"
#include <random>

/**
  @brief RRT路径规划器

*/
namespace rmp::path_planner {
class RRTPlanner : public PathPlanner {
public:
  using Node = rmp::common::structure::Node<int>;

  RRTPlanner(costmap_2d::Costmap2DROS *costmap);
  ~RRTPlanner();

  bool plan(const common::geometry::Point3d &start,
            const common::geometry::Point3d &end,
            common::geometry::Points3d *path,
            common::geometry::Points3d *expand) override;

protected:
  // 采样
  Node _generateRandomNode();
  // 计算最邻近
  Node _findNearestNode(std::unordered_map<int, Node> &list, const Node &node);
  // 计算是否到达目标点
  bool _checkGoal(const Node &new_node);

private:
  int sample_points_ = 1500;
  double sample_max_distance_ = 30.0;
  double optimization_radius_ = 20.0;
  double optimization_sample_probability_ = 0.05;
  Node start_;
  Node goal_;
  std::unordered_map<int, Node> sample_list_;
  // 碰撞检测器

  // 随机数引擎，作为成员，构造函数仅初始化一次
  std::mt19937 rng_eng_;
  std::uniform_real_distribution<float> prob_dist_;
};
} // namespace rmp::path_planner
