#include <gtest/gtest.h>

#include <cmath>

#include "../include/local_planner/local_planner_node.h"

#include <boost/algorithm/string.hpp>

// Stateless tests:
// Create some hardcoded scan data of obstacles in different positions
// For each one check that the planner response is correct
using namespace avoidance;

TEST(LocalPlannerNodeTests, failsafe) {
  ros::Time::init();
  LocalPlannerNode Node;
  bool planner_is_healthy = true;
  bool hover = false;

  Node.position_received_ = 1;

  // Node.hover_point_.pose.position.x = 1.0f;
  //   Node.hover_point_.pose.position.y = 1.0f;
  //   Node.hover_point_.pose.position.z = 1.0f;

  avoidance::LocalPlannerNodeConfig config =
      avoidance::LocalPlannerNodeConfig::__getDefault__();

  ros::Duration since_last_cloud = ros::Duration(0.0);
  ros::Duration since_start = ros::Duration(0.0);
  Node.never_run_ = 0;

  for (int i = 0; i < 3; i++) {
    Node.checkFailsafe(since_last_cloud, since_start, planner_is_healthy,
                       hover);
    since_last_cloud = since_last_cloud + ros::Duration(0.2f);
    since_start = since_start + ros::Duration(0.2f);
    std::cout << "status_msg_.state " << (int)Node.status_msg_.state
              << " planner_is_healthy " << planner_is_healthy << std::endl;
    EXPECT_TRUE(planner_is_healthy);
    EXPECT_EQ(Node.status_msg_.state, (int)MAV_STATE::MAV_STATE_ACTIVE);
  }

  for (int i = 3; i < 75; i++) {
    Node.checkFailsafe(since_last_cloud, since_start, planner_is_healthy,
                       hover);
    since_last_cloud = since_last_cloud + ros::Duration(0.2f);
    since_start = since_start + ros::Duration(0.2f);
    std::cout << "status_msg_.state " << (int)Node.status_msg_.state
              << " planner_is_healthy " << planner_is_healthy << std::endl;
    EXPECT_TRUE(planner_is_healthy);
    EXPECT_EQ(Node.status_msg_.state, (int)MAV_STATE::MAV_STATE_CRITICAL);
  }

  for (int i = 75; i < 91; i++) {
    Node.checkFailsafe(since_last_cloud, since_start, planner_is_healthy,
                       hover);
    since_last_cloud = since_last_cloud + ros::Duration(0.2f);
    since_start = since_start + ros::Duration(0.2f);
    std::cout << "status_msg_.state " << (int)Node.status_msg_.state
              << " planner_is_healthy " << planner_is_healthy << std::endl;
    EXPECT_FALSE(planner_is_healthy);
    EXPECT_EQ(Node.status_msg_.state,
              (int)MAV_STATE::MAV_STATE_FLIGHT_TERMINATION);
  }
}
