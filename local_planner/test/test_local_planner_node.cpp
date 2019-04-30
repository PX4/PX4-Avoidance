#include <gtest/gtest.h>

#include "../include/local_planner/local_planner_node.h"

using namespace avoidance;

TEST(LocalPlannerNodeTests, failsafe) {
  ros::Time::init();
  ros::NodeHandle nh("~");
  ros::NodeHandle nh_private("");
  LocalPlannerNode Node(nh, nh_private, false);
  bool planner_is_healthy = true;
  bool hover = false;

  Node.position_received_ = true;
  Node.never_run_ = false;
  Node.setSystemStatus(MAV_STATE::MAV_STATE_ACTIVE);

  avoidance::LocalPlannerNodeConfig config =
      avoidance::LocalPlannerNodeConfig::__getDefault__();

  ros::Duration since_last_cloud = ros::Duration(0.0);
  ros::Duration since_start = ros::Duration(0.0);
  double time_increment = 0.2f;
  int active_n_iter = std::ceil(config.timeout_critical_ / time_increment);
  int critical_n_iter = std::ceil(config.timeout_termination_ / time_increment);

  for (int i = 0; i < active_n_iter; i++) {
    Node.checkFailsafe(since_last_cloud, since_start, planner_is_healthy,
                       hover);
    since_last_cloud = since_last_cloud + ros::Duration(time_increment);
    since_start = since_start + ros::Duration(time_increment);
    EXPECT_TRUE(planner_is_healthy);
    EXPECT_EQ(Node.getSystemStatus(), MAV_STATE::MAV_STATE_ACTIVE);
  }

  for (int i = active_n_iter; i < critical_n_iter; i++) {
    Node.checkFailsafe(since_last_cloud, since_start, planner_is_healthy,
                       hover);
    since_last_cloud = since_last_cloud + ros::Duration(time_increment);
    since_start = since_start + ros::Duration(time_increment);
    EXPECT_TRUE(planner_is_healthy);
    EXPECT_EQ(Node.getSystemStatus(), MAV_STATE::MAV_STATE_CRITICAL);
  }

  for (int i = critical_n_iter; i < 91; i++) {
    Node.checkFailsafe(since_last_cloud, since_start, planner_is_healthy,
                       hover);
    since_last_cloud = since_last_cloud + ros::Duration(time_increment);
    since_start = since_start + ros::Duration(time_increment);
    EXPECT_FALSE(planner_is_healthy);
    EXPECT_EQ(Node.getSystemStatus(), MAV_STATE::MAV_STATE_FLIGHT_TERMINATION);
  }
}
