#include "path_planner_factory.h"

using namespace rmp::path_planner;
bool PathPlannerFactory::createPlanner(ros::NodeHandle &nh,
                                       costmap_2d::Costmap2DROS *costmap_ros,
                                       PlannerProps &planner_props) {
  std::string planner_name;
  nh.param("planner_name", planner_name, (std::string) "astar"); // planner name

  if (planner_name == "astar") {
    planner_props.planner_ptr = std::make_shared<AStarPathPlanner>(costmap_ros);
    planner_props.planner_type = GRAPH_PLANNER;
  } else if (planner_name == "dijkstra") {
    planner_props.planner_ptr =
        std::make_shared<AStarPathPlanner>(costmap_ros, true);
    planner_props.planner_type = GRAPH_PLANNER;
  } else if (planner_name == "gbfs") {
    planner_props.planner_ptr =
        std::make_shared<AStarPathPlanner>(costmap_ros, false, true);
    planner_props.planner_type = GRAPH_PLANNER;
  } else if (planner_name == "rrt") {
    planner_props.planner_ptr = std::make_shared<RRTPlanner>(costmap_ros);
    planner_props.planner_type = SAMPLE_PLANNER;
  }
  //    else if (planner_name == "jps") {
  //     planner_props.planner_ptr =
  //     std::make_shared<JPSPathPlanner>(costmap_ros);
  //     planner_props.planner_type = GRAPH_PLANNER;
  //   }
  //   } else if (planner_name == "dstar") {
  //     planner_props.planner_ptr =
  //     std::make_shared<DStarPathPlanner>(costmap_ros);
  //     planner_props.planner_type = GRAPH_PLANNER;
  //   } else if (planner_name == "lpa_star") {
  //     planner_props.planner_ptr =
  //         std::make_shared<LPAStarPathPlanner>(costmap_ros);
  //     planner_props.planner_type = GRAPH_PLANNER;
  //   } else if (planner_name == "dstar_lite") {
  //     planner_props.planner_ptr =
  //         std::make_shared<DStarLitePathPlanner>(costmap_ros);
  //     planner_props.planner_type = GRAPH_PLANNER;
  //   } else if (planner_name == "voronoi") {
  //     planner_props.planner_ptr = std::make_shared<VoronoiPathPlanner>(
  //         costmap_ros,
  //         costmap_ros->getLayeredCostmap()->getCircumscribedRadius());
  //     planner_props.planner_type = GRAPH_PLANNER;
  //   } else if (planner_name == "lazy") {
  //     planner_props.planner_ptr =
  //     std::make_shared<LazyPathPlanner>(costmap_ros);
  //     planner_props.planner_type = GRAPH_PLANNER;
  //   } else if (planner_name == "theta_star") {
  //     planner_props.planner_ptr =
  //         std::make_shared<ThetaStarPathPlanner>(costmap_ros);
  //     planner_props.planner_type = GRAPH_PLANNER;
  //   } else if (planner_name == "lazy_theta_star") {
  //     planner_props.planner_ptr =
  //         std::make_shared<LazyThetaStarPathPlanner>(costmap_ros);
  //     planner_props.planner_type = GRAPH_PLANNER;
  //   } else if (planner_name == "s_theta_star") {
  //     planner_props.planner_ptr =
  //         std::make_shared<SThetaStarPathPlanner>(costmap_ros);
  //     planner_props.planner_type = GRAPH_PLANNER;
  //   }
  //   else if (planner_name == "hybrid_astar") {
  //     planner_props.planner_ptr =
  //         std::make_shared<HybridAStarPathPlanner>(costmap_ros);
  //     planner_props.planner_type = GRAPH_PLANNER;
  //   } else if (planner_name == "rrt") {
  //     planner_props.planner_ptr =
  //     std::make_shared<RRTPathPlanner>(costmap_ros);
  //     planner_props.planner_type = SAMPLE_PLANNER;
  //   } else if (planner_name == "rrt_star") {
  //     planner_props.planner_ptr =
  //         std::make_shared<RRTStarPathPlanner>(costmap_ros);
  //     planner_props.planner_type = SAMPLE_PLANNER;
  //   } else if (planner_name == "rrt_connect") {
  //     planner_props.planner_ptr =
  //         std::make_shared<RRTConnectPathPlanner>(costmap_ros);
  //     planner_props.planner_type = SAMPLE_PLANNER;
  //   } else if (planner_name == "informed_rrt") {
  //     planner_props.planner_ptr =
  //         std::make_shared<InformedRRTStarPathPlanner>(costmap_ros);
  //     planner_props.planner_type = SAMPLE_PLANNER;
  //   }
  //   else if (planner_name == "aco") {
  //     planner_props.planner_ptr =
  //     std::make_shared<ACOPathPlanner>(costmap_ros);
  //     planner_props.planner_type = EVOLUTION_PLANNER;
  //   } else if (planner_name == "pso") {
  //     planner_props.planner_ptr =
  //     std::make_shared<PSOPathPlanner>(costmap_ros);
  //     planner_props.planner_type = EVOLUTION_PLANNER;
  //   } else if (planner_name == "ga") {
  //     planner_props.planner_ptr =
  //     std::make_shared<GAPathPlanner>(costmap_ros);
  //     planner_props.planner_type = EVOLUTION_PLANNER;
  //   }
  //   else {
  //     ROS_ERROR_STREAM("Unknown planner name: " << planner_name);
  //     return false;
  //   }

  ROS_INFO_STREAM("Using path planner: " << planner_name);
  return true;
}