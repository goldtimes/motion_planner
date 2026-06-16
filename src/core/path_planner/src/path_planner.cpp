#include "path_planner.h"

using namespace rmp::path_planner;
using namespace rmp::common;
using namespace rmp::common::geometry;
// 完成规划器需要的公共接口

PathPlanner::PathPlanner(costmap_2d::Costmap2DROS *costmap)
    : costmap_ros_(costmap), nx_(costmap_ros_->getCostmap()->getSizeInCellsX()),
      ny_(costmap_ros_->getCostmap()->getSizeInCellsY()) {
  map_size_ = nx_ * ny_;
  costmap_ = costmap_ros_->getCostmap();
  double obstacle_inflation_factor = 0.5;
  collision_checker_ = std::make_shared<CollisionChecker>(
      costmap_ros_, obstacle_inflation_factor);
}

costmap_2d::Costmap2D *PathPlanner::getCostMap() const { return costmap_; }

int PathPlanner::getMapSize() const { return map_size_; }

int PathPlanner::grid2Index(int x, int y) {
  // 将grid的索引变成一维的索引
  return y * nx_ + x;
}

void PathPlanner::index2Grid(int i, int &x, int &y) {
  // 将一维的索引变成grid的索引
  x = static_cast<int>(i % nx_);
  y = static_cast<int>(i / nx_);
}

bool PathPlanner::world2Map(double wx, double wy, double &mx, double &my) {
  // costmap 是以左下角为原点的，所以需要将 world map 转换为 costmap 坐标
  if (wx < costmap_->getOriginX() || wy < costmap_->getOriginY()) {
    return false;
  }
  mx = (wx - costmap_->getOriginX()) / costmap_->getResolution();
  my = (wy - costmap_->getOriginY()) / costmap_->getResolution();
  if (mx < nx_ && my < ny_)
    return true;
  return false;
}

void PathPlanner::map2World(double mx, double my, double &wx, double &wy) {
  // costmap 是以左下角为原点的，所以需要将 costmap 坐标转换为 world map 坐标
  wx = costmap_->getOriginX() + (mx + 0.5) * costmap_->getResolution();
  wy = costmap_->getOriginY() + (my + 0.5) * costmap_->getResolution();
}

/**
 * @brief Inflate the boundary of costmap into obstacles to prevent cross
 * planning
 */
void PathPlanner::outlineMap() {
  // 将 costmap 边界膨胀为障碍物，防止路径规划时穿过边界
  // 遍历第一行
  auto pc = costmap_->getCharMap();
  for (int i = 0; i < nx_; i++) {
    *pc++ = costmap_2d::LETHAL_OBSTACLE;
  }
  // 遍历最后一行
  pc = costmap_->getCharMap() + (ny_ - 1) * nx_;
  for (int i = 0; i < nx_; i++) {
    *pc++ = costmap_2d::LETHAL_OBSTACLE;
  }
  // 遍历第一列
  pc = costmap_->getCharMap();
  for (int i = 0; i < ny_; i++) {
    *pc = costmap_2d::LETHAL_OBSTACLE;
    pc += nx_;
  }

  // 遍历最后一列
  pc = costmap_->getCharMap() + (nx_ - 1);
  for (int i = 0; i < ny_; i++) {
    *pc = costmap_2d::LETHAL_OBSTACLE;
    pc += nx_;
  }
}

bool PathPlanner::validityCheck(double wx, double wy, double &mx, double &my) {
  if (!world2Map(wx, wy, mx, my)) {
    ROS_WARN("The robot's position is off the global costmap. Planning will "
             "always "
             "fail, are you sure the robot "
             "has been properly localized?");
    return false;
  }
  return true;
}

bool PathPlanner::resample(const common::geometry::Points3d &path,
                           common::geometry::Points3d *path_resample,
                           double sample_ratio) {
  return true;
}
