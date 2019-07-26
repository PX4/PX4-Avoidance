#include "global_planner/global_planner_node.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "global_planner_node");

  ros::NodeHandle nh("~");
  ros::NodeHandle nh_private("");

  global_planner::GlobalPlannerNode global_planner_node(nh, nh_private);

  ros::spin();
  return 0;
}
