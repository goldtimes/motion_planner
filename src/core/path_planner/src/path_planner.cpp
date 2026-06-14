#include "path_planner.h"

using namespace rmp::path_planner;

// 完成规划器需要的公共接口

PathPlanner::PathPlanner(costmap_2d::Costmap2DROS *costmap)
    : costmap_ros_(costmap), nx_(costmap_ros_->getCostmap()->getSizeInCellsX()),
      ny_(costmap_ros_->getCostmap()->getSizeInCellsY()) {
  map_size_ = nx_ * ny_;
  costmap_ = costmap_ros_->getCostmap();
}
