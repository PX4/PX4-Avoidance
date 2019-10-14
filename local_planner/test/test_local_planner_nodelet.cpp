#include <gtest/gtest.h>

#include "../include/local_planner/local_planner_nodelet.h"

using namespace avoidance;

TEST(LocalPlannerNodeletTests, failsafe) {
  ros::Time::init();
  LocalPlannerNodelet Node;
  Node.InitializeNodelet();
  bool hover = false;

  Node.position_received_ = true;
  Node.setSystemStatus(MAV_STATE::MAV_STATE_ACTIVE);

  avoidance::LocalPlannerNodeConfig config = avoidance::LocalPlannerNodeConfig::__getDefault__();

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

TEST(LocalPlannerNodeletTests, mission_item_speed) {
  ros::Time::init();
  LocalPlannerNodelet Node;
  Node.InitializeNodelet();

  mavros_msgs::WaypointList waypoint_list{};
  mavros_msgs::Waypoint wp1;
  wp1.frame = 3;
  wp1.command = 22;
  wp1.is_current = true;
  wp1.autocontinue = true;
  wp1.param1 = 15.f;
  wp1.param2 = 0.f;
  wp1.param3 = 0.f;
  wp1.param4 = NAN;
  wp1.x_lat = 47.397743f;
  wp1.y_long = 8.545715f;
  wp1.z_alt = 15.f;
  waypoint_list.waypoints.push_back(wp1);

  mavros_msgs::Waypoint wp2;
  wp2.frame = 3;
  wp2.command = 16;
  wp2.is_current = false;
  wp2.autocontinue = true;
  wp2.param1 = 0.f;
  wp2.param2 = 0.f;
  wp2.param3 = 0.f;
  wp2.param4 = NAN;
  wp2.x_lat = 47.397743f;
  wp2.y_long = 8.545908f;
  wp2.z_alt = 15.f;
  waypoint_list.waypoints.push_back(wp2);

  mavros_msgs::Waypoint wp3;
  wp3.frame = 2;
  wp3.command = 205;
  wp3.is_current = false;
  wp3.autocontinue = true;
  wp3.param1 = 0.f;
  wp3.param2 = 0.f;
  wp3.param3 = 90.f;
  wp3.param4 = 0.f;
  wp3.x_lat = 0.f;
  wp3.y_long = 0.f;
  wp3.z_alt = 2.f;
  waypoint_list.waypoints.push_back(wp3);

  mavros_msgs::Waypoint wp4;
  wp4.frame = 2;
  wp4.command = 2000;
  wp4.is_current = false;
  wp4.autocontinue = true;
  wp4.param1 = 0.f;
  wp4.param2 = 0.f;
  wp4.param3 = 1.f;
  wp4.param4 = 0.f;
  wp4.x_lat = NAN;
  wp4.y_long = NAN;
  wp4.z_alt = NAN;
  waypoint_list.waypoints.push_back(wp4);

  mavros_msgs::Waypoint wp5;
  wp5.frame = 2;
  wp5.command = 178;
  wp5.is_current = false;
  wp5.autocontinue = true;
  wp5.param1 = 1.f;
  wp5.param2 = 16.f;
  wp5.param3 = -1.f;
  wp5.param4 = 0.f;
  wp5.x_lat = 0.f;
  wp5.y_long = 0.f;
  wp5.z_alt = 0.f;
  waypoint_list.waypoints.push_back(wp5);

  mavros_msgs::Waypoint wp6;
  wp6.frame = 3;
  wp6.command = 16;
  wp6.is_current = false;
  wp6.autocontinue = true;
  wp6.param1 = 0.f;
  wp6.param2 = 0.f;
  wp6.param3 = 0.f;
  wp6.param4 = NAN;
  wp6.x_lat = 47.397751f;
  wp6.y_long = 8.546041f;
  wp6.z_alt = 15.f;
  waypoint_list.waypoints.push_back(wp6);

  mavros_msgs::Waypoint wp7;
  wp7.frame = 3;
  wp7.command = 16;
  wp7.is_current = false;
  wp7.autocontinue = true;
  wp7.param1 = 0.f;
  wp7.param2 = 0.f;
  wp7.param3 = 0.f;
  wp7.param4 = NAN;
  wp7.x_lat = 47.397736f;
  wp7.y_long = 8.546223f;
  wp7.z_alt = 15.f;
  waypoint_list.waypoints.push_back(wp7);

  mavros_msgs::Waypoint wp8;
  wp8.frame = 2;
  wp8.command = 178;
  wp8.is_current = false;
  wp8.autocontinue = true;
  wp8.param1 = 1.f;
  wp8.param2 = 20.f;
  wp8.param3 = -1.f;
  wp8.param4 = 0.f;
  wp8.x_lat = 0.f;
  wp8.y_long = 0.f;
  wp8.z_alt = 0.f;
  waypoint_list.waypoints.push_back(wp8);

  mavros_msgs::Waypoint wp9;
  wp9.frame = 3;
  wp9.command = 21;
  wp9.is_current = false;
  wp9.autocontinue = true;
  wp9.param1 = 0.f;
  wp9.param2 = 0.f;
  wp9.param3 = 0.f;
  wp9.param4 = NAN;
  wp9.x_lat = 47.397747f;
  wp9.y_long = 8.546336f;
  wp9.z_alt = 0.f;

  waypoint_list.waypoints.push_back(wp9);
  std::vector<int> is_current_index_list{0, 1, 5, 6, 7};
  std::vector<float> mission_item_speed_truth{NAN, NAN, 16, 16, 20};

  for (int i = 0; i < is_current_index_list.size(); i++) {
    if ((i - 1) >= 0) {
      waypoint_list.waypoints[is_current_index_list[i - 1]].is_current = 0;
    }
    waypoint_list.waypoints[is_current_index_list[i]].is_current = 1;
    Node.avoidance_node_->missionCallback(waypoint_list);

    float mission_item_speed = Node.avoidance_node_->getMissionItemSpeed();
    float truth = mission_item_speed_truth[i];
    if (i < 2) {
      ASSERT_FALSE(std::isfinite(mission_item_speed));
    } else {
      EXPECT_FLOAT_EQ(truth, mission_item_speed);
    }
  }
}
