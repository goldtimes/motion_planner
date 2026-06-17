cmake_minimum_required(VERSION 3.0.2)
project(teb_controller)

get_filename_component(PROJECT_ROOT_DIR ${CMAKE_SOURCE_DIR} DIRECTORY)
include(${PROJECT_ROOT_DIR}/3rd/conanbuildinfo.cmake)
conan_basic_setup()

find_package(catkin REQUIRED COMPONENTS
  angles
  costmap_2d
  dynamic_reconfigure
  geometry_msgs
  nav_core
  nav_msgs
  pluginlib
  roscpp
  tf2
  tf2_geometry_msgs
  tf2_ros
  base_local_planner
  controller
  system_config
)

find_package(Eigen3 REQUIRED)

# Dynamic reconfigure
generate_dynamic_reconfigure_options(
  cfg/TEBController.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES teb_controller
  CATKIN_DEPENDS controller system_config base_local_planner
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/teb_controller.cpp
  src/teb_optimizer.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_include_directories(${PROJECT_NAME} PUBLIC
  ${CONAN_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CONAN_LIBS}
)
