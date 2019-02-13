#include "local_planner/local_planner_node.h"

#include <boost/algorithm/string.hpp>

int main(int argc, char** argv) {
  using namespace avoidance;
  ros::init(argc, argv, "local_planner_node");

  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  LocalPlannerNode Node(nh, nh_private);

  ros::Duration(2).sleep();
  avoidanceOutput planner_output;

  std::thread worker(&LocalPlannerNode::threadFunction, &Node);

  Node.should_exit_ = true;
  Node.data_ready_cv_.notify_all();
  worker.join();
  return 0;
}
