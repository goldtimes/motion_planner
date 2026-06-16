#include "graph_planner/astar_planner.h"

using namespace rmp::path_planner;
using namespace rmp::common::structure;
using namespace rmp::common::geometry;

// 8-directional A* path planner
// 定义了8个方向的运动，坐标和g值
std::vector<AStarPathPlanner::Node> AStarPathPlanner::motions_ = {
    {0, 1, 1.0},     {0, -1, 1.0},     {1, 0, 1.0},      {-1, 0, 1.0},
    {1, 1, sqrt(2)}, {1, -1, sqrt(2)}, {-1, 1, sqrt(2)}, {-1, -1, sqrt(2)}};

AStarPathPlanner::AStarPathPlanner(costmap_2d::Costmap2DROS *costmap,
                                   bool is_dijkstra, bool gbfs)
    : PathPlanner(costmap) {
  if (!(is_dijkstra && gbfs)) {
    is_dijkstra_ = is_dijkstra;
    gbfs_ = gbfs;
  } else {
    is_dijkstra_ = false;
    gbfs_ = false;
  }
}

AStarPathPlanner::~AStarPathPlanner() {}

/**
    A*算法的流程了
    1. 检查下标是否合法
*/
bool AStarPathPlanner::plan(const common::geometry::Point3d &start,
                            const common::geometry::Point3d &end,
                            common::geometry::Points3d *path,
                            common::geometry::Points3d *expand) {
  double m_start_x, m_start_y, m_goal_x, m_goal_y;
  if (validityCheck(start.x(), start.y(), m_start_x, m_start_y)) {
    return false;
  }
  if (validityCheck(end.x(), end.y(), m_goal_x, m_goal_y)) {
    return false;
  }
  // 清空路径和扩展点
  path->clear();
  expand->clear();

  // 规划的起点，终点合法
  // 构建第一个起点,终点Node
  Node start_node(m_start_x, m_start_y);
  Node goal_node(m_goal_x, m_goal_y);
  start_node.set_id(grid2Index(m_start_x, m_start_y));
  goal_node.set_id(grid2Index(m_goal_x, m_goal_y));

  // open_list 最小堆
  std::priority_queue<Node, std::vector<Node>, Node::compare_cost> open_list;
  // closed_list 存放以及访问过的节点
  std::unordered_map<int, Node> closed_list;

  // 将起点放入open_list
  open_list.push(start_node);
  // 不断的循环，直到open_list为空
  while (!open_list.empty()) {
    // 取出最小堆中的节点，也就是f = g +h 最小的节点
    auto current = open_list.top();
    open_list.pop();
    // 如果已经在closed_list中，直接跳过
    if (closed_list.find(current.id()) != closed_list.end()) {
      continue;
    }
    // 添加到closed_list中
    closed_list.insert({current.id(), current});
    expand->emplace_back(current.x(), current.y());
    // 判断是否为终点
    if (current == goal_node) {
      // 到达终点，返回true
      const auto &backtrace =
          _convertClosedListToPath<Node>(closed_list, start_node, goal_node);
      for (auto iter = backtrace.rbegin(); iter != backtrace.rend(); ++iter) {
        // convert to world frame
        double wx, wy;
        costmap_->mapToWorld(iter->x(), iter->y(), wx, wy);
        path->emplace_back(wx, wy);
      }
      return true;
    }
    // 遍历周围的八个点
    for (const auto &motion : motions_) {
      // 计算新的节点
      auto new_node = current + motion;
      new_node.set_g(current.g() + motion.g());
      new_node.set_id(grid2Index(new_node.x(), new_node.y()));
      // 如果周围的点，已经在closed_list中，直接跳过
      if (closed_list.find(new_node.id()) != closed_list.end()) {
        continue;
      }
      // 合法的new_node，设置父亲节点
      new_node.set_pid(current.id());
      // 判断新节点是否为障碍物，超出地图边界，或者是否为未知区域，则跳过
      if ((new_node.id() < 0) || (new_node.id() >= map_size_) ||
          (costmap_->getCharMap()[new_node.id()] >=
               costmap_2d::LETHAL_OBSTACLE * 0.5 &&
           costmap_->getCharMap()[new_node.id()] >=
               costmap_->getCharMap()[current.id()])) {
        continue;
      }

      if (gbfs_) {
        // 贪婪模式，则将所有的g设置为0
        new_node.set_g(0.0);
      }
      if (!is_dijkstra_) {
        // 标准的A*，设置启发函数，dijkstra没有启发函数，这里采用欧式距离
        new_node.set_h(std::hypot(new_node.x() - goal_node.x(),
                                  new_node.y() - goal_node.y()));
      }
      // 将new_node放入open_list
      open_list.push(new_node);
    }
  }
  return false;
}



// 
