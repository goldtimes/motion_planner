#pragma once

#include "node.h"
#include "path_planner.h"

namespace rmp::path_planner {

class AStarPathPlanner : public PathPlanner {
public:
  AStarPathPlanner(costmap_2d::Costmap2DROS *costmap, bool is_dijkstra = false,
                   bool gbfs = false);
  ~AStarPathPlanner();

  /**
   * @brief A* implementation
   * @param start          start node
   * @param goal           goal node
   * @param path           The resulting path in (x, y, theta)
   * @param expand         containing the node been search during the process
   * @return true if path found, else false
   */
  bool path(const common::geometry::Point3d &start,
            const common::geometry::Point3d &end,
            common::geometry::Points3d *path,
            common::geometry::Points3d *expand) override;

private:
  bool is_dijkstra_;
  bool gbfs_;
  using Node = rmp::common::structure::Node<int>;
  static std::vector<Node> motions_;
};

} // namespace rmp::path_planner