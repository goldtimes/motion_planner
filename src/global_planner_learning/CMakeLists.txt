cmake_minimum_required(VERSION 3.0.2)
project(global_planner_learning)

# 开启编译命令导出（便于 IDE 索引）
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## 查找 catkin 以及所有依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
)

## 声明包信息
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    nav_msgs
    geometry_msgs
    visualization_msgs
    tf2_ros
  DEPENDS
)

## 头文件路径
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## ============================================================
## 编译：可执行节点
## ============================================================
add_executable(${PROJECT_NAME}_node
  src/global_planner_node.cpp
  src/astar_planner.cpp
  src/dijkstra_planner.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

## ============================================================
## 安装规则
## ============================================================
install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
