#include "safe_landing_planner/safe_landing_planner_node.hpp"

int main(int argc, char **argv) {
  using namespace avoidance;
  ros::init(argc, argv, "safe_landing_planner_node");
  ros::NodeHandle nh("~");
  SafeLandingPlannerNode NodeSLP(nh);
  NodeSLP.startNode();

  while (ros::ok()) {
    ros::spin();
  }

  NodeSLP.worker_.join();
  return 0;
}
