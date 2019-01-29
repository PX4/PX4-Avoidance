#include <gtest/gtest.h>

#include "../src/nodes/common.h"
#include "../src/nodes/waypoint_generator.h"

using namespace avoidance;

class WaypointGeneratorTests : public ::testing::Test {
 public:
  WaypointGenerator waypoint_generator;
  avoidanceOutput avoidance_output;
  geometry_msgs::PoseStamped position;
  geometry_msgs::PoseStamped goal;
  geometry_msgs::TwistStamped velocity;
  bool stay = false;

  void SetUp() override {
    ros::Time::init();
    ros::Time time = ros::Time::now();

    avoidance_output.waypoint_type = direct;
    avoidance_output.reach_altitude = true;
    avoidance_output.obstacle_ahead = false;
    avoidance_output.min_speed = 1.0;
    avoidance_output.max_speed = 3.0;
    avoidance_output.velocity_sigmoid_slope = 3.0;
    avoidance_output.last_path_time = time - ros::Duration(0.5);
    avoidance_output.back_off_point.x = 0.4f;
    avoidance_output.back_off_point.y = 0.6f;
    avoidance_output.back_off_point.z = 2.2f;

    avoidance_output.back_off_start_point.x = 0.0f;
    avoidance_output.back_off_start_point.y = 0.0f;
    avoidance_output.back_off_start_point.z = 2.0f;

    PolarPoint p_pol = histogramIndexToPolar(15, 35, 6, 0.0);
    avoidance_output.costmap_direction_e = p_pol.e;
    avoidance_output.costmap_direction_z = p_pol.z;

    geometry_msgs::Point n0;
    n0.x = 0.0f;
    n0.y = 0.0f;
    n0.z = 2.0f;
    geometry_msgs::Point n1;
    n1.x = 0.8f;
    n1.y = sqrtf(1 - (n1.x * n1.x));
    n1.z = 2.0f;
    geometry_msgs::Point n2;
    n2.x = 1.3f;
    n2.y = n1.y + sqrtf(1 - powf(n2.x - n1.x, 2));
    n2.z = 2.0f;
    geometry_msgs::Point n3;
    n3.x = 2.2f;
    n3.y = n2.y + sqrtf(1 - powf(n3.x - n2.x, 2));
    n3.z = 2.0f;
    geometry_msgs::Point n4;
    n4.x = 2.9f;
    n4.y = n3.y + sqrtf(1 - powf(n4.x - n3.x, 2));
    n4.z = 2.0f;
    geometry_msgs::Point n5;
    n5.x = 3.5f;
    n5.y = n4.y + sqrtf(1 - powf(n5.x - n4.x, 2));
    n5.z = 2.0f;
    avoidance_output.path_node_positions = {n5, n4, n3, n2, n1, n0};

    position.pose.position.x = 0.0;
    position.pose.position.y = 0.0;
    position.pose.position.z = 2.0;
    position.pose.orientation.x = 0.0;
    position.pose.orientation.y = 0.0;
    position.pose.orientation.z = 0.0;
    position.pose.orientation.w = 1.0f;
    avoidance_output.pose = position;

    goal.pose.position.x = 10.0;
    goal.pose.position.y = 3.0;
    goal.pose.position.z = 2.0;

    goal.pose.orientation.x = 0.0;
    goal.pose.orientation.y = 0.0;
    goal.pose.orientation.z = 0.0;
    goal.pose.orientation.w = 1.0f;

    velocity.twist.linear.x = 2.5f;
    velocity.twist.linear.y = 3.0f;
    velocity.twist.linear.z = 0.0f;

    velocity.twist.angular.x = 0.0f;
    velocity.twist.angular.y = 0.0f;
    velocity.twist.angular.z = 0.0f;

    waypoint_generator.updateState(position, goal, velocity, stay, time);
    waypoint_generator.setPlannerInfo(avoidance_output);
    waypoint_generator.setFOV(270.0, 45.0);
    waypoint_generator.param_.goal_acceptance_radius_in = 0.5f;
    waypoint_generator.param_.goal_acceptance_radius_out = 1.5f;
    waypoint_generator.param_.factor_close_to_goal_start_speed_limitation =
        3.0f;
    waypoint_generator.param_.factor_close_to_goal_stop_speed_limitation = 4.0f;
    waypoint_generator.param_.max_speed_close_to_goal_factor = 0.1f;
    waypoint_generator.param_.min_speed_close_to_goal = 0.5f;
  }
  void TearDown() override {}
};

TEST_F(WaypointGeneratorTests, goFastTest) {
  // GIVEN: a waypoint of type goFast

  // WHEN: we generate waypoints
  waypointResult result = waypoint_generator.getWaypoints();

  // THEN: we expect a waypoint in between the vehicle curent position and the
  // goal
  EXPECT_FLOAT_EQ(0.9578262852f, result.goto_position.x);
  EXPECT_FLOAT_EQ(0.2873478856f, result.goto_position.y);
  EXPECT_FLOAT_EQ(2.0f, result.goto_position.z);

  EXPECT_FLOAT_EQ(1.0f, result.adapted_goto_position.x);
  EXPECT_FLOAT_EQ(0.3f, result.adapted_goto_position.y);
  EXPECT_FLOAT_EQ(2.0f, result.adapted_goto_position.z);

  EXPECT_FLOAT_EQ(0.00041298225, result.position_waypoint.pose.position.x);
  EXPECT_FLOAT_EQ(0.0003515787, result.position_waypoint.pose.position.y);
  EXPECT_FLOAT_EQ(0.000320, result.position_waypoint.pose.position.z);

  EXPECT_FLOAT_EQ(0.0, result.position_waypoint.pose.orientation.x);
  EXPECT_FLOAT_EQ(0.0, result.position_waypoint.pose.orientation.y);
  EXPECT_FLOAT_EQ(0.34536624, result.position_waypoint.pose.orientation.z);
  EXPECT_FLOAT_EQ(0.938468, result.position_waypoint.pose.orientation.w);

  EXPECT_FLOAT_EQ(0.00041298225, result.velocity_waypoint.linear.x);
  EXPECT_FLOAT_EQ(0.0003515787, result.velocity_waypoint.linear.y);
  EXPECT_FLOAT_EQ(-1.999680, result.velocity_waypoint.linear.z);

  EXPECT_FLOAT_EQ(0.0, result.velocity_waypoint.angular.x);
  EXPECT_FLOAT_EQ(0.0, result.velocity_waypoint.angular.y);
  EXPECT_FLOAT_EQ(0.352629, result.velocity_waypoint.angular.z);
}

TEST_F(WaypointGeneratorTests, reachAltitudeTest) {
  // GIVEN: a waypoint of type goFast and the vehicle has not yet reached the
  // goal altiude
  avoidance_output.reach_altitude = false;
  avoidance_output.offboard_pose.pose.position.x = 0.0;
  avoidance_output.offboard_pose.pose.position.y = 0.0;
  avoidance_output.offboard_pose.pose.position.z = 0.0;

  goal.pose.position.z = 5.0;
  ros::Time time = ros::Time::now();
  waypoint_generator.setPlannerInfo(avoidance_output);
  waypoint_generator.updateState(position, goal, velocity, stay, time);

  // WHEN: we generate waypoints
  waypointResult result = waypoint_generator.getWaypoints();

  // THEN: we expect a position waypoint with the same x-y coordinates as the
  // vehicle position and a z cordinate in between the vehicle and the goal
  // altitude
  EXPECT_FLOAT_EQ(0.0f, result.goto_position.x);
  EXPECT_FLOAT_EQ(0.0f, result.goto_position.y);
  EXPECT_FLOAT_EQ(3.0f, result.goto_position.z);

  EXPECT_FLOAT_EQ(0.0f, result.adapted_goto_position.x);
  EXPECT_FLOAT_EQ(0.0f, result.adapted_goto_position.y);
  EXPECT_FLOAT_EQ(3.0f, result.adapted_goto_position.z);

  EXPECT_FLOAT_EQ(0.0, result.position_waypoint.pose.position.x);
  EXPECT_FLOAT_EQ(0.0, result.position_waypoint.pose.position.y);
  EXPECT_FLOAT_EQ(3.0, result.position_waypoint.pose.position.z);

  EXPECT_FLOAT_EQ(0.0, result.position_waypoint.pose.orientation.x);
  EXPECT_FLOAT_EQ(0.0, result.position_waypoint.pose.orientation.y);
  EXPECT_FLOAT_EQ(0.0, result.position_waypoint.pose.orientation.z);
  EXPECT_FLOAT_EQ(1.0, result.position_waypoint.pose.orientation.w);

  EXPECT_FLOAT_EQ(0.0, result.velocity_waypoint.linear.x);
  EXPECT_FLOAT_EQ(0.0, result.velocity_waypoint.linear.y);
  EXPECT_FLOAT_EQ(1.0, result.velocity_waypoint.linear.z);

  EXPECT_FLOAT_EQ(0.0, result.velocity_waypoint.angular.x);
  EXPECT_FLOAT_EQ(0.0, result.velocity_waypoint.angular.y);
  EXPECT_FLOAT_EQ(0.0, result.velocity_waypoint.angular.z);
}

TEST_F(WaypointGeneratorTests, goBackTest) {
  // GIVEN: a waypoint of type goBack
  avoidance_output.waypoint_type = goBack;
  avoidance_output.reach_altitude = true;
  waypoint_generator.setPlannerInfo(avoidance_output);

  // WHEN: we generate waypoints
  waypointResult result = waypoint_generator.getWaypoints();

  // THEN: we expect the position waypoint to be moving away from the goal
  EXPECT_FLOAT_EQ(-0.27735f, result.goto_position.x);
  EXPECT_FLOAT_EQ(-0.416025f, result.goto_position.y);
  EXPECT_FLOAT_EQ(avoidance_output.back_off_start_point.z,
                  result.goto_position.z);

  EXPECT_FLOAT_EQ(-0.27735009789, result.position_waypoint.pose.position.x);
  EXPECT_FLOAT_EQ(-0.41602513194, result.position_waypoint.pose.position.y);
  EXPECT_FLOAT_EQ(2.0, result.position_waypoint.pose.position.z);

  EXPECT_FLOAT_EQ(0.0, result.position_waypoint.pose.orientation.x);
  EXPECT_FLOAT_EQ(0.0, result.position_waypoint.pose.orientation.y);
  EXPECT_FLOAT_EQ(0.0, result.position_waypoint.pose.orientation.z);
  EXPECT_FLOAT_EQ(1.0, result.position_waypoint.pose.orientation.w);

  EXPECT_FLOAT_EQ(-0.27735009789, result.velocity_waypoint.linear.x);
  EXPECT_FLOAT_EQ(-0.41602513194, result.velocity_waypoint.linear.y);
  EXPECT_FLOAT_EQ(0.0, result.velocity_waypoint.linear.z);

  EXPECT_FLOAT_EQ(0.0, result.velocity_waypoint.angular.x);
  EXPECT_FLOAT_EQ(0.0, result.velocity_waypoint.angular.y);
  EXPECT_FLOAT_EQ(0.0, result.velocity_waypoint.angular.z);
}

TEST_F(WaypointGeneratorTests, hoverTest) {
  // GIVEN: a waypoint of type hover
  avoidance_output.waypoint_type = hover;
  waypoint_generator.setPlannerInfo(avoidance_output);

  // WHEN: we generate waypoints
  waypointResult result = waypoint_generator.getWaypoints();

  // THEN: we expect the position waypoint to be the same as the current vehicle
  // position
  EXPECT_FLOAT_EQ(position.pose.position.x, result.goto_position.x);
  EXPECT_FLOAT_EQ(position.pose.position.y, result.goto_position.y);
  EXPECT_FLOAT_EQ(position.pose.position.z, result.goto_position.z);

  EXPECT_FLOAT_EQ(0.00025298225, result.position_waypoint.pose.position.x);
  EXPECT_FLOAT_EQ(0.00030357868, result.position_waypoint.pose.position.y);
  EXPECT_FLOAT_EQ(0.000320, result.position_waypoint.pose.position.z);

  EXPECT_FLOAT_EQ(0.0, result.position_waypoint.pose.orientation.x);
  EXPECT_FLOAT_EQ(0.0, result.position_waypoint.pose.orientation.y);
  EXPECT_FLOAT_EQ(0.42415538, result.position_waypoint.pose.orientation.z);
  EXPECT_FLOAT_EQ(0.9055894, result.position_waypoint.pose.orientation.w);

  EXPECT_FLOAT_EQ(0.00025298225, result.velocity_waypoint.linear.x);
  EXPECT_FLOAT_EQ(0.00030357868, result.velocity_waypoint.linear.y);
  EXPECT_FLOAT_EQ(-1.999680, result.velocity_waypoint.linear.z);

  EXPECT_FLOAT_EQ(0.0, result.velocity_waypoint.angular.x);
  EXPECT_FLOAT_EQ(0.0, result.velocity_waypoint.angular.y);
  EXPECT_FLOAT_EQ(0.438029, result.velocity_waypoint.angular.z);
}

TEST_F(WaypointGeneratorTests, costmapTest) {
  // GIVEN: a waypoint of type costmap
  avoidance_output.waypoint_type = costmap;
  waypoint_generator.setPlannerInfo(avoidance_output);

  // WHEN: we generate waypoints
  waypointResult result = waypoint_generator.getWaypoints();

  // THEN: we expect the position waypoint to be in the direction defined by the
  // elevation and azimuth costmap
  EXPECT_FLOAT_EQ(0.5438926261f, result.goto_position.x);
  EXPECT_FLOAT_EQ(0.8375211991f, result.goto_position.y);
  EXPECT_FLOAT_EQ(2.052335956f, result.goto_position.z);

  EXPECT_FLOAT_EQ(0.32640055, result.adapted_goto_position.x);
  EXPECT_FLOAT_EQ(0.50261277, result.adapted_goto_position.y);
  EXPECT_FLOAT_EQ(2.03140830994f, result.adapted_goto_position.z);

  EXPECT_FLOAT_EQ(0.00030520631, result.position_waypoint.pose.position.x);
  EXPECT_FLOAT_EQ(0.00038399675, result.position_waypoint.pose.position.y);
  EXPECT_FLOAT_EQ(0.00032502529, result.position_waypoint.pose.position.z);

  EXPECT_FLOAT_EQ(0.0, result.position_waypoint.pose.orientation.x);
  EXPECT_FLOAT_EQ(0.0, result.position_waypoint.pose.orientation.y);
  EXPECT_FLOAT_EQ(0.43461663, result.position_waypoint.pose.orientation.z);
  EXPECT_FLOAT_EQ(0.90061557, result.position_waypoint.pose.orientation.w);

  EXPECT_FLOAT_EQ(0.00030520631, result.velocity_waypoint.linear.x);
  EXPECT_FLOAT_EQ(0.00038399675, result.velocity_waypoint.linear.y);
  EXPECT_FLOAT_EQ(-1.99967497357, result.velocity_waypoint.linear.z);

  EXPECT_FLOAT_EQ(0.0, result.velocity_waypoint.angular.x);
  EXPECT_FLOAT_EQ(0.0, result.velocity_waypoint.angular.y);
  EXPECT_FLOAT_EQ(0.44961253, result.velocity_waypoint.angular.z);
}

TEST_F(WaypointGeneratorTests, trypathTest) {
  // GIVEN: a waypoint of type tryPath
  avoidance_output.waypoint_type = tryPath;
  waypoint_generator.setPlannerInfo(avoidance_output);

  // WHEN: we generate waypoints
  waypointResult result = waypoint_generator.getWaypoints();

  // THEN: we expect the position waypoint to be in the direction of the closest
  // tree node to the current vehicle position
  EXPECT_FLOAT_EQ(0.8f, result.goto_position.x);
  EXPECT_FLOAT_EQ(0.6f, result.goto_position.y);
  EXPECT_FLOAT_EQ(2.0f, result.goto_position.z);

  EXPECT_FLOAT_EQ(0.83522457, result.adapted_goto_position.x);
  EXPECT_FLOAT_EQ(0.62641847, result.adapted_goto_position.y);
  EXPECT_FLOAT_EQ(2.0f, result.adapted_goto_position.z);

  EXPECT_FLOAT_EQ(0.00038661817, result.position_waypoint.pose.position.x);
  EXPECT_FLOAT_EQ(0.00040380566, result.position_waypoint.pose.position.y);
  EXPECT_FLOAT_EQ(0.00032, result.position_waypoint.pose.position.z);

  EXPECT_FLOAT_EQ(0.0, result.position_waypoint.pose.orientation.x);
  EXPECT_FLOAT_EQ(0.0, result.position_waypoint.pose.orientation.y);
  EXPECT_FLOAT_EQ(0.39270377, result.position_waypoint.pose.orientation.z);
  EXPECT_FLOAT_EQ(0.91966504, result.position_waypoint.pose.orientation.w);

  EXPECT_FLOAT_EQ(0.00038661817, result.velocity_waypoint.linear.x);
  EXPECT_FLOAT_EQ(0.00040380566, result.velocity_waypoint.linear.y);
  EXPECT_FLOAT_EQ(-1.99968, result.velocity_waypoint.linear.z);

  EXPECT_FLOAT_EQ(0.0, result.velocity_waypoint.angular.x);
  EXPECT_FLOAT_EQ(0.0, result.velocity_waypoint.angular.y);
  EXPECT_FLOAT_EQ(0.4035697, result.velocity_waypoint.angular.z);
}
