#include "avoidance/avoidance_node.h"

using namespace avoidance;

int main(int argc, char** argv) {
  ros::init(argc, argv, "avoidance_node");
  ros::NodeHandle nh("~");
  ros::NodeHandle nh_private("");

  AvoidanceNode Node(nh, nh_private);
  return 0;
}