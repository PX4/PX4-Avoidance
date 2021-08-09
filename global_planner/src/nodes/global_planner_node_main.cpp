#include "global_planner/global_planner_node.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto global_planner_nd = std::make_shared<global_planner::GlobalPlannerNode>();
  rclcpp::spin(global_planner_nd);
  return 0;
}
