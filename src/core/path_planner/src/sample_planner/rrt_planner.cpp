#include "sample_planner/rrt_planner.h"

using namespace rmp::path_planner;

RRTPlanner::RRTPlanner(costmap_2d::Costmap2DROS *costmap)
    : PathPlanner(costmap) {
  ROS_INFO("RRTPlanner initialized");
  std::random_device rd;
  rng_eng_.seed(rd());
  prob_dist_ = std::uniform_real_distribution<float>(0.f, 1.f);
}

RRTPlanner::~RRTPlanner() {}

bool RRTPlanner::plan(const common::geometry::Point3d &start,
                      const common::geometry::Point3d &end,
                      common::geometry::Points3d *path,
                      common::geometry::Points3d *expand) {
  // 检查起点和终点是否合法
  double m_start_x, m_start_y, m_goal_x, m_goal_y;
  if (!validityCheck(start.x(), start.y(), m_start_x, m_start_y)) {
    return false;
  }
  if (!validityCheck(end.x(), end.y(), m_goal_x, m_goal_y)) {
    return false;
  }
  path->clear();
  expand->clear();
  sample_list_.clear();

  // 创建起点，终点
  start_.set_x(m_start_x);
  start_.set_y(m_start_y);
  start_.set_id(grid2Index(m_start_x, m_start_y));

  goal_.set_x(m_goal_x);
  goal_.set_y(m_goal_y);
  goal_.set_id(grid2Index(m_goal_x, m_goal_y));

  // 将起点添加到采样列表中
  sample_list_.insert({start_.id(), start_});
  // 将起点添加到扩展点中
  expand->emplace_back(start_.x(), start_.y(), 0);

  // RRT算法的流程
  // 迭代次数/采样个数
  for (int iter = 0; iter < sample_points_; iter++) {
    // 生成随机节点
    Node sample_node = _generateRandomNode();
    // 检查这个随机的节点是否合法
    // obstacle
    if (costmap_->getCharMap()[sample_node.id()] >=
        costmap_2d::LETHAL_OBSTACLE * 0.5) {
      continue;
    }

    // visited
    if (sample_list_.find(sample_node.id()) != sample_list_.end()) {
      continue;
    }
    // 在树中查找采样点的最近邻近点，然后让采样点用一定的步长往采样点靠拢
    Node new_node = _findNearestNode(sample_list_, sample_node);
    // 检查new_node是否没有碰撞
    if (new_node.id() == -1) {
      continue;
    } else {
      // 如果new_node没有碰撞，就将new_node添加到采样列表中
      sample_list_.insert(std::make_pair(new_node.id(), new_node));
      expand->emplace_back(new_node.x(), new_node.y(), new_node.pid());
    }
    // 检查new_node是否到达目标点
    if (_checkGoal(new_node)) {
      // 回溯路径
      const auto &backtrace =
          _convertClosedListToPath<Node>(sample_list_, start_, goal_);
      for (auto iter = backtrace.rbegin(); iter != backtrace.rend(); ++iter) {
        // convert to world frame
        double wx, wy;
        costmap_->mapToWorld(iter->x(), iter->y(), wx, wy);
        path->emplace_back(wx, wy);
      }
      return true;
    }
  }
  return false;
}

RRTPlanner::Node RRTPlanner::_generateRandomNode() {
  // 启发式采样：一定概率直接采样目标点
  if (prob_dist_(rng_eng_) > optimization_sample_probability_) {
    std::uniform_int_distribution<int> distr(0, map_size_ - 1);
    const int id = distr(rng_eng_);
    int x, y;
    index2Grid(id, x, y);
    return Node(x, y, 0, 0, id, 0);

  } else {
    // 采样目标点
    return Node(goal_.x(), goal_.y(), 0, 0, goal_.id(), 0);
  }
}
// 计算最邻近
RRTPlanner::Node
RRTPlanner::_findNearestNode(std::unordered_map<int, Node> &list,
                             const Node &node) {
  Node nearest_node;
  Node new_node(node);
  double min_dist = std::numeric_limits<double>::max();
  // 遍历树节点
  for (const auto &p : list) {
    // 计算树到采样点的欧式距离
    // double dx = p.second.x() - node.x();
    // double dy = p.second.y() - node.y();
    // double manhattan_dist = std::fabs(dx) + std::fabs(dy);
    double new_dist =
        std::hypot(p.second.x() - node.x(), p.second.y() - node.y());
    // 更新最近邻近点
    if (new_dist < min_dist) {
      min_dist = new_dist;
      nearest_node = p.second;
      // 将这个最近邻近点作为新节点的父节点
      new_node.set_pid(nearest_node.id());
      // 更新新节点的g值 new_node 到最近邻的代价
      new_node.set_g(new_dist + p.second.g());
    }
  }
  // 上面已经找到最近邻近点。如果最近邻和采样点的距离大于threshold,不能直接连接步长太大，容易穿过障碍物、运动不连续
  // 于是沿着两点连线，从最近节点向外截取一段长度为 max_dist
  // 的点作为真正新增节点
  if (min_dist > sample_max_distance_) {
    double theta = std::atan2(new_node.y() - nearest_node.y(),
                              new_node.x() - nearest_node.x());
    new_node.set_x(nearest_node.x() +
                   static_cast<int>(sample_max_distance_ * cos(theta)));
    new_node.set_y(nearest_node.y() +
                   static_cast<int>(sample_max_distance_ * sin(theta)));
    new_node.set_id(grid2Index(new_node.x(), new_node.y()));
    new_node.set_g(sample_max_distance_ + nearest_node.g());
  }
  // 判断new_node 和 nearest_node 之间连线是否与障碍物碰撞
  auto isCollision = [&](const Node &node1, const Node &node2) {
    return rmp::common::geometry::CollisionChecker::BresenhamCollisionDetection(
        node1, node2, [&](const Node &node) {
          return costmap_->getCharMap()[grid2Index(node.x(), node.y())] >=
                 costmap_2d::LETHAL_OBSTACLE * 0.5;
        });
  };
  if (isCollision(new_node, nearest_node)) {
    new_node.set_id(-1);
  }
  return new_node;
}
/**
 * @brief Check if goal is reachable from current node
 * @param new_node Current node
 * @return bool value of whether goal is reachable from current node
 */
// 计算是否到达目标点
bool RRTPlanner::_checkGoal(const Node &new_node) {
  auto dist_ = std::hypot(new_node.x() - goal_.x(), new_node.y() - goal_.y());
  if (dist_ > sample_max_distance_)
    return false;

  auto isCollision = [&](const Node &node1, const Node &node2) {
    return rmp::common::geometry::CollisionChecker::BresenhamCollisionDetection(
        node1, node2, [&](const Node &node) {
          return costmap_->getCharMap()[grid2Index(node.x(), node.y())] >=
                 costmap_2d::LETHAL_OBSTACLE * 0.5;
        });
  };

  if (!isCollision(new_node, goal_)) {
    Node goal(goal_.x(), goal_.y(), dist_ + new_node.g(), 0,
              grid2Index(goal_.x(), goal_.y()), new_node.id());
    sample_list_.insert(std::make_pair(goal.id(), goal));
    return true;
  }
  return false;
}
