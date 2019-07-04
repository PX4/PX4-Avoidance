#include <gtest/gtest.h>

#include "../include/local_planner/local_planner_node.h"

using namespace avoidance;

TEST(LocalPlannerNodeTests, failsafe) {
  ros::Time::init();
  ros::NodeHandle nh("~");
  ros::NodeHandle nh_private("");
  LocalPlannerNode Node(nh, nh_private, false);
  bool hover = false;

  Node.position_received_ = true;
  Node.setSystemStatus(MAV_STATE::MAV_STATE_ACTIVE);

  avoidance::LocalPlannerNodeConfig config =
      avoidance::LocalPlannerNodeConfig::__getDefault__();

  ros::Duration since_last_cloud = ros::Duration(0.0);
  ros::Duration since_start = ros::Duration(config.timeout_startup_);
  double time_increment = 0.2f;
  int active_n_iter = std::ceil(config.timeout_critical_ / time_increment);
  int critical_n_iter = std::ceil(config.timeout_termination_ / time_increment);

  for (int i = 0; i < active_n_iter; i++) {
    Node.checkFailsafe(since_last_cloud, since_start, hover);
    since_last_cloud = since_last_cloud + ros::Duration(time_increment);
    since_start = since_start + ros::Duration(time_increment);
    EXPECT_EQ(Node.getSystemStatus(), MAV_STATE::MAV_STATE_ACTIVE);
  }

  for (int i = active_n_iter; i < critical_n_iter; i++) {
    Node.checkFailsafe(since_last_cloud, since_start, hover);
    since_last_cloud = since_last_cloud + ros::Duration(time_increment);
    since_start = since_start + ros::Duration(time_increment);
    EXPECT_EQ(Node.getSystemStatus(), MAV_STATE::MAV_STATE_CRITICAL);
  }

  for (int i = critical_n_iter; i < 91; i++) {
    Node.checkFailsafe(since_last_cloud, since_start, hover);
    since_last_cloud = since_last_cloud + ros::Duration(time_increment);
    since_start = since_start + ros::Duration(time_increment);
    EXPECT_EQ(Node.getSystemStatus(), MAV_STATE::MAV_STATE_FLIGHT_TERMINATION);
  }
}

TEST(LocalPlannerNodeTests, removeNaNAndUpdateFOV) {
  ros::Time::init();
  ros::NodeHandle nh("~");
  ros::NodeHandle nh_private("");
  LocalPlannerNode Node(nh, nh_private, false);
  bool hover = false;

  // GIVEN: two point clouds, one including NANs, one without
  pcl::PointCloud<pcl::PointXYZ> pc_no_nan, pc_with_nan;
  for (float x = -40.f; x <= 40.f; x += 1.f) {
    for (float y = -30.f; y <= 30.f; y += 1.f) {
      pcl::PointXYZ p(x, y, 15.f);
      pc_no_nan.push_back(p);
      pc_with_nan.push_back(p);
      pc_with_nan.push_back(pcl::PointXYZ(NAN, x, y));  // garbage point
    }
  }
  pc_with_nan.is_dense = false;
  pc_no_nan.is_dense = true;

  // WHEN: we filter these clouds
  // Node.removeNaNAndUpdateFOV(pc_no_nan);
  // Node.removeNaNAndUpdateFOV(pc_with_nan);

  // THEN: we expect the clouds to not contain NANs, be dense and reflect
  // the extrema and minima
  EXPECT_EQ(pc_no_nan.size(), pc_with_nan.size());
  EXPECT_TRUE(pc_no_nan.is_dense);
  EXPECT_TRUE(pc_with_nan.is_dense);
}
