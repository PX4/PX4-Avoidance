#include <gtest/gtest.h>

#include "../include/local_planner/common.h"
#include "../include/local_planner/waypoint_generator.h"

using namespace avoidance;

class WaypointGeneratorTests : public ::testing::Test,
                               public WaypointGenerator {
 public:
  avoidanceOutput avoidance_output;
  geometry_msgs::PoseStamped position;
  geometry_msgs::PoseStamped goal;
  geometry_msgs::TwistStamped velocity;
  bool stay = false;
  ros::Time time = ros::Time(0.33);

  ros::Time getSystemTime() override { return time; }

  void SetUp() override {
    ros::Time::init();

    avoidance_output.waypoint_type = direct;
    avoidance_output.obstacle_ahead = false;
    avoidance_output.velocity_around_obstacles = 1.0;
    avoidance_output.velocity_far_from_obstacles = 3.0;
    avoidance_output.last_path_time = ros::Time(0.28);
    avoidance_output.back_off_point.x = 0.4;
    avoidance_output.back_off_point.y = 0.6;
    avoidance_output.back_off_point.z = 2.2;

    avoidance_output.back_off_start_point.x = 0.0;
    avoidance_output.back_off_start_point.y = 0.0;
    avoidance_output.back_off_start_point.z = 2.0;

    PolarPoint p_pol = histogramIndexToPolar(15, 35, 6, 0.0);
    avoidance_output.costmap_direction_e = p_pol.e;
    avoidance_output.costmap_direction_z = p_pol.z;

    avoidance_output.offboard_pose.pose.position.x = 0.0;
    avoidance_output.offboard_pose.pose.position.y = 0.0;
    avoidance_output.offboard_pose.pose.position.z = 0.0;

    geometry_msgs::Point n0;
    n0.x = 0.0;
    n0.y = 0.0;
    n0.z = 2.0;
    geometry_msgs::Point n1;
    n1.x = 0.8;
    n1.y = sqrtf(1 - (n1.x * n1.x));
    n1.z = 2.0;
    geometry_msgs::Point n2;
    n2.x = 1.3;
    n2.y = n1.y + sqrtf(1 - powf(n2.x - n1.x, 2));
    n2.z = 2.0;
    geometry_msgs::Point n3;
    n3.x = 2.2;
    n3.y = n2.y + sqrtf(1 - powf(n3.x - n2.x, 2));
    n3.z = 2.0;
    geometry_msgs::Point n4;
    n4.x = 2.9;
    n4.y = n3.y + sqrtf(1 - powf(n4.x - n3.x, 2));
    n4.z = 2.0;
    geometry_msgs::Point n5;
    n5.x = 3.5;
    n5.y = n4.y + sqrtf(1 - powf(n5.x - n4.x, 2));
    n5.z = 2.0;
    avoidance_output.path_node_positions = {n5, n4, n3, n2, n1, n0};

    position.pose.position.x = 0.0;
    position.pose.position.y = 0.0;
    position.pose.position.z = 0.0;
    position.pose.orientation.x = 0.0;
    position.pose.orientation.y = 0.0;
    position.pose.orientation.z = 0.0;
    position.pose.orientation.w = 1.0;

    goal.pose.position.x = 10.0;
    goal.pose.position.y = 3.0;
    goal.pose.position.z = 2.0;

    goal.pose.orientation.x = 0.0;
    goal.pose.orientation.y = 0.0;
    goal.pose.orientation.z = 0.0;
    goal.pose.orientation.w = 1.0;

    velocity.twist.linear.x = 0.0;
    velocity.twist.linear.y = 0.0;
    velocity.twist.linear.z = 0.0;

    velocity.twist.angular.x = 0.0;
    velocity.twist.angular.y = 0.0;
    velocity.twist.angular.z = 0.0;

    updateState(position, goal, velocity, stay);
    setPlannerInfo(avoidance_output);
    setFOV(270.0, 45.0);

    param_.goal_acceptance_radius_in = 0.5;
    param_.goal_acceptance_radius_out = 1.5;
  }
  void TearDown() override {}
};

TEST_F(WaypointGeneratorTests, reachAltitudeTest) {
  // GIVEN: a waypoint of type goFast and the vehicle has not yet reached the
  // goal altiude
  avoidance_output.waypoint_type = reachHeight;
  goal.pose.position.z = 5.0;
  setPlannerInfo(avoidance_output);
  double time_sec = 0.0;
  float goto_to_goal_prev = 1000.0f;
  float adapted_to_goal_prev = 1000.0f;
  float pos_sp_to_goal_prev = 1000.0f;

  // WHEN: we generate the first waypoint
  time = ros::Time(time_sec);
  updateState(position, goal, velocity, stay);
  waypointResult result = getWaypoints();

  // THEN: we expect the goto position to point straight up
  EXPECT_NEAR(position.pose.position.x, result.goto_position.x, 0.1);
  EXPECT_NEAR(position.pose.position.y, result.goto_position.y, 0.1);
  EXPECT_LT(position.pose.position.z, result.goto_position.z);

  // THEN: we expect the adapted goto position to be between goal and drone in z
  EXPECT_GT(result.adapted_goto_position.z, position.pose.position.z);
  EXPECT_LT(result.adapted_goto_position.z, goal.pose.position.z);

  // THEN: we expect the adapted goto position to be close to the drone location
  // in xy
  EXPECT_NEAR(position.pose.position.x, result.adapted_goto_position.x, 0.1);
  EXPECT_NEAR(position.pose.position.y, result.adapted_goto_position.y, 0.1);

  // THEN: we expect the smoothed goto position to be the same as the drone
  // location
  // (first iteration of smoothing)
  EXPECT_NEAR(position.pose.position.x, result.smoothed_goto_position.x, 0.1);
  EXPECT_NEAR(position.pose.position.y, result.smoothed_goto_position.y, 0.1);
  EXPECT_NEAR(position.pose.position.z, result.smoothed_goto_position.z, 0.1);

  // THEN: we expect the smoothed goto position to be the position waypoint,
  // since
  // smoothing was enabled
  EXPECT_EQ(result.smoothed_goto_position.x,
            result.position_waypoint.pose.position.x);
  EXPECT_EQ(result.smoothed_goto_position.y,
            result.position_waypoint.pose.position.y);
  EXPECT_EQ(result.smoothed_goto_position.z,
            result.position_waypoint.pose.position.z);

  // WHEN: we generate subsequent waypoints
  for (size_t i = 0; i < 10; i++) {
    // calculate new vehicle position
    time_sec += 0.03;
    time = ros::Time(time_sec);
    updateState(position, goal, velocity, stay);
    waypointResult result = getWaypoints();

    // THEN: we expect the goto location to point straight up
    EXPECT_NEAR(position.pose.position.x, result.goto_position.x, 0.1);
    EXPECT_NEAR(position.pose.position.y, result.goto_position.y, 0.1);
    EXPECT_LT(position.pose.position.z, result.goto_position.z);

    // THEN: we expect the adapted goto position to be between goal and drone in
    // z
    EXPECT_GT(result.adapted_goto_position.z, position.pose.position.z);
    EXPECT_LT(result.adapted_goto_position.z, goal.pose.position.z);

    // THEN: we expect the adapted goto position to be close to the drone
    // location in xy
    EXPECT_NEAR(position.pose.position.x, result.adapted_goto_position.x, 0.1);
    EXPECT_NEAR(position.pose.position.y, result.adapted_goto_position.y, 0.1);

    // THEN: we expect the smoothed goto position to be between the drone
    // and the adapted goto position in z
    EXPECT_LT(position.pose.position.z, result.smoothed_goto_position.z);
    EXPECT_GT(result.adapted_goto_position.z, result.smoothed_goto_position.z);

    // THEN: we expect the smoothed goto position to be the position waypoint,
    // since
    // smoothing was enabled
    EXPECT_EQ(result.smoothed_goto_position.x,
              result.position_waypoint.pose.position.x);
    EXPECT_EQ(result.smoothed_goto_position.y,
              result.position_waypoint.pose.position.y);
    EXPECT_EQ(result.smoothed_goto_position.z,
              result.position_waypoint.pose.position.z);

    // THEN: we expect the z component of the waypoint to move closer to goal.z
    float goto_to_goal =
        std::abs(goal.pose.position.z - result.goto_position.z);
    float adapted_to_goal =
        std::abs(goal.pose.position.z - result.adapted_goto_position.z);
    float pos_sp_to_goal = std::abs(goal.pose.position.z -
                                    result.position_waypoint.pose.position.z);
    ASSERT_LT(goto_to_goal, goto_to_goal_prev);
    ASSERT_LT(adapted_to_goal, adapted_to_goal_prev);
    ASSERT_LT(pos_sp_to_goal, pos_sp_to_goal_prev);
    goto_to_goal_prev = goto_to_goal;
    adapted_to_goal_prev = adapted_to_goal;
    pos_sp_to_goal_prev = pos_sp_to_goal;

    // Update the state for next iteration, assume we get half-way from current
    // location toward the position setpoint
    Eigen::Vector3f pos = toEigen(position.pose.position);
    Eigen::Vector3f pos_to_pos_sp =
        (toEigen(result.position_waypoint.pose.position) - pos) * 0.5f;
    Eigen::Vector3f new_pos = pos + pos_to_pos_sp;
    position.pose.position = toPoint(new_pos);

    // THEN: we expect the angle between the position and velocity waypoint to
    // be small
    Eigen::Vector3f vel_sp(result.velocity_waypoint.linear.x,
                           result.velocity_waypoint.linear.y,
                           result.velocity_waypoint.linear.z);
    float angle_pos_vel_sp = std::atan2(pos_to_pos_sp.cross(vel_sp).norm(),
                                        pos_to_pos_sp.dot(vel_sp));
    EXPECT_NEAR(0.0, angle_pos_vel_sp, 1.0);
  }
}

TEST_F(WaypointGeneratorTests, goStraightTest) {
  // GIVEN: a waypoint of type goStraight
  avoidance_output.waypoint_type = direct;
  setPlannerInfo(avoidance_output);

  float goto_to_goal_prev = 1000.0f;
  float adapted_to_goal_prev = 1000.0f;
  float pos_sp_to_goal_prev = 1000.0f;
  double time_sec = time.toSec();

  // WHEN: we generate waypoints
  for (size_t i = 0; i < 10; i++) {
    time_sec += 0.03;
    time = ros::Time(time_sec);
    updateState(position, goal, velocity, stay);

    waypointResult result = getWaypoints();
    float goto_to_goal =
        (toEigen(goal.pose.position) - toEigen(result.goto_position)).norm();
    float adapted_to_goal =
        (toEigen(goal.pose.position) - toEigen(result.adapted_goto_position))
            .norm();
    float pos_sp_to_goal = (toEigen(goal.pose.position) -
                            toEigen(result.position_waypoint.pose.position))
                               .norm();
    // THEN: we expect the waypoints to move closer to the goal
    ASSERT_LE(goto_to_goal, goto_to_goal_prev);
    ASSERT_LE(adapted_to_goal, adapted_to_goal_prev);
    ASSERT_LE(pos_sp_to_goal, pos_sp_to_goal_prev);
    goto_to_goal_prev = goto_to_goal;
    adapted_to_goal_prev = adapted_to_goal;
    pos_sp_to_goal_prev = pos_sp_to_goal;

    // Assume we get halfway from current position to the setpoint
    Eigen::Vector3f pos = toEigen(position.pose.position);
    Eigen::Vector3f pos_to_pos_sp =
        (toEigen(result.position_waypoint.pose.position) - pos) * 0.5;
    Eigen::Vector3f vel_sp(result.velocity_waypoint.linear.x,
                           result.velocity_waypoint.linear.y,
                           result.velocity_waypoint.linear.z);
    // THEN: we expect the angle between the position and velocity waypoint to
    // be small
    float angle_pos_vel_sp = std::atan2(pos_to_pos_sp.cross(vel_sp).norm(),
                                        pos_to_pos_sp.dot(vel_sp));
    EXPECT_NEAR(0.0, angle_pos_vel_sp, 1.0);

    // calculate new vehicle position
    Eigen::Vector3f new_pos = pos + pos_to_pos_sp;
    position.pose.position = toPoint(new_pos);
  }
}

TEST_F(WaypointGeneratorTests, goBackTest) {
  // GIVEN: a waypoint of type goBack (adapted_goto_position not filled in this
  // case)
  avoidance_output.waypoint_type = goBack;
  setPlannerInfo(avoidance_output);

  float goto_to_goal_prev = -1.0f;
  float pos_sp_to_goal_prev = -1.0f;
  double time_sec = 0.33;

  // WHEN: we generate waypoints
  for (size_t i = 0; i < 10; i++) {
    waypointResult result = getWaypoints();
    float goto_to_goal =
        (toEigen(goal.pose.position) - toEigen(result.goto_position)).norm();
    float pos_sp_to_goal = (toEigen(goal.pose.position) -
                            toEigen(result.position_waypoint.pose.position))
                               .norm();
    // THEN: we expect the waypoints to move further away from the goal
    ASSERT_GT(goto_to_goal, goto_to_goal_prev);
    ASSERT_GT(pos_sp_to_goal, pos_sp_to_goal_prev);
    goto_to_goal_prev = goto_to_goal;
    pos_sp_to_goal_prev = pos_sp_to_goal;

    // Assume we get halfway from current position to the setpoint
    Eigen::Vector3f pos = toEigen(position.pose.position);
    Eigen::Vector3f pos_to_pos_sp =
        (toEigen(result.position_waypoint.pose.position) - pos) * 0.5;
    Eigen::Vector3f vel_sp(result.velocity_waypoint.linear.x,
                           result.velocity_waypoint.linear.y,
                           result.velocity_waypoint.linear.z);

    // THEN: we expect the angle between the position and velocity waypoint to
    // be small
    float angle_pos_vel_sp = std::atan2(pos_to_pos_sp.cross(vel_sp).norm(),
                                        pos_to_pos_sp.dot(vel_sp));
    EXPECT_NEAR(0.0, angle_pos_vel_sp, 1.0);

    // calculate new vehicle position
    Eigen::Vector3f new_pos = pos + pos_to_pos_sp;
    position.pose.position = toPoint(new_pos);
    time_sec += 0.033;
    time = ros::Time(time_sec);
    updateState(position, goal, velocity, stay);
  }
}

TEST_F(WaypointGeneratorTests, hoverTest) {
  // GIVEN: a waypoint of type hover

  // first run one the waypoint generator such that smoothed_goto_location_ gets
  // initialize
  setPlannerInfo(avoidance_output);
  double time_sec = 0.0;
  time = ros::Time(time_sec);
  updateState(position, goal, velocity, stay);
  waypointResult result = getWaypoints();

  avoidance_output.waypoint_type = hover;
  setPlannerInfo(avoidance_output);
  time_sec += 0.033;
  time = ros::Time(time_sec);
  updateState(position, goal, velocity, stay);

  // WHEN: we generate waypoints
  result = getWaypoints();

  // THEN: we expect the position waypoint to be the same as the current vehicle
  // position
  EXPECT_NEAR(position.pose.position.x, result.goto_position.x, 0.001);
  EXPECT_NEAR(position.pose.position.y, result.goto_position.y, 0.001);
  EXPECT_NEAR(position.pose.position.z, result.goto_position.z, 0.001);

  EXPECT_NEAR(position.pose.position.x,
              result.position_waypoint.pose.position.x, 0.01);
  EXPECT_NEAR(position.pose.position.y,
              result.position_waypoint.pose.position.y, 0.01);
  EXPECT_NEAR(position.pose.position.z,
              result.position_waypoint.pose.position.z, 0.01);

  EXPECT_NEAR(0.0, result.position_waypoint.pose.orientation.x, 0.1);
  EXPECT_NEAR(0.0, result.position_waypoint.pose.orientation.y, 0.1);
  EXPECT_NEAR(0.0, result.position_waypoint.pose.orientation.z, 0.1);
  EXPECT_NEAR(1.0, result.position_waypoint.pose.orientation.w, 0.1);

  EXPECT_NEAR(0.0, result.velocity_waypoint.linear.x, 0.01);
  EXPECT_NEAR(0.0, result.velocity_waypoint.linear.y, 0.01);
  EXPECT_NEAR(0.0, result.velocity_waypoint.linear.z, 0.01);

  EXPECT_NEAR(0.0, result.velocity_waypoint.angular.x, 0.1);
  EXPECT_NEAR(0.0, result.velocity_waypoint.angular.y, 0.1);
  EXPECT_NEAR(0.0, result.velocity_waypoint.angular.z, 0.1);
}

TEST_F(WaypointGeneratorTests, costmapTest) {
  // GIVEN: a waypoint of type costmap
  avoidance_output.waypoint_type = costmap;
  setPlannerInfo(avoidance_output);

  float goto_to_goal_prev = 1000.0f;
  float adapted_to_goal_prev = 1000.0f;
  float pos_sp_to_goal_prev = 1000.0f;
  double time_sec = 0.33;

  // WHEN: we generate waypoints
  for (size_t i = 0; i < 10; i++) {
    waypointResult result = getWaypoints();
    float goto_to_goal =
        (toEigen(goal.pose.position) - toEigen(result.goto_position)).norm();
    float adapted_to_goal =
        (toEigen(goal.pose.position) - toEigen(result.adapted_goto_position))
            .norm();
    float pos_sp_to_goal = (toEigen(goal.pose.position) -
                            toEigen(result.position_waypoint.pose.position))
                               .norm();
    // THEN: we expect the waypoints to move closer to the goal
    ASSERT_LE(goto_to_goal, goto_to_goal_prev);
    ASSERT_LE(adapted_to_goal, adapted_to_goal_prev);
    ASSERT_LE(pos_sp_to_goal, pos_sp_to_goal_prev);
    goto_to_goal_prev = goto_to_goal;
    adapted_to_goal_prev = adapted_to_goal;
    pos_sp_to_goal_prev = pos_sp_to_goal;

    // Assume we get halfway from current position to the setpoint
    Eigen::Vector3f pos = toEigen(position.pose.position);
    Eigen::Vector3f pos_to_pos_sp =
        (toEigen(result.position_waypoint.pose.position) - pos) * 0.5;
    Eigen::Vector3f vel_sp(result.velocity_waypoint.linear.x,
                           result.velocity_waypoint.linear.y,
                           result.velocity_waypoint.linear.z);

    // THEN: we expect the angle between the position and velocity waypoint to
    // be small
    float angle_pos_vel_sp = std::atan2(pos_to_pos_sp.cross(vel_sp).norm(),
                                        pos_to_pos_sp.dot(vel_sp));
    EXPECT_NEAR(0.0, angle_pos_vel_sp, 1.0);

    // calculate new vehicle position
    Eigen::Vector3f new_pos = pos + pos_to_pos_sp;
    position.pose.position = toPoint(new_pos);
    time_sec += 0.03;
    time = ros::Time(time_sec);
    updateState(position, goal, velocity, stay);
  }
}

TEST_F(WaypointGeneratorTests, trypathTest) {
  // GIVEN: a waypoint of type tryPath
  avoidance_output.waypoint_type = tryPath;
  setPlannerInfo(avoidance_output);

  float goto_to_goal_prev = 1000.0f;
  float adapted_to_goal_prev = 1000.0f;
  float pos_sp_to_goal_prev = 1000.0f;
  double time_sec = 0.33;

  // WHEN: we generate waypoints
  for (size_t i = 0; i < 10; i++) {
    time_sec += 0.033;
    time = ros::Time(time_sec);
    updateState(position, goal, velocity, stay);

    waypointResult result = getWaypoints();
    float goto_to_goal =
        (toEigen(goal.pose.position) - toEigen(result.goto_position)).norm();
    float adapted_to_goal =
        (toEigen(goal.pose.position) - toEigen(result.adapted_goto_position))
            .norm();
    float pos_sp_to_goal = (toEigen(goal.pose.position) -
                            toEigen(result.position_waypoint.pose.position))
                               .norm();

    ASSERT_LE(goto_to_goal, goto_to_goal_prev);
    ASSERT_LE(adapted_to_goal, adapted_to_goal_prev);
    ASSERT_LE(pos_sp_to_goal, pos_sp_to_goal_prev);
    goto_to_goal_prev = goto_to_goal;
    adapted_to_goal_prev = adapted_to_goal;
    pos_sp_to_goal_prev = pos_sp_to_goal;

    // Assume we get halfway from current position to the setpoint
    Eigen::Vector3f pos = toEigen(position.pose.position);
    Eigen::Vector3f pos_to_pos_sp =
        (toEigen(result.position_waypoint.pose.position) - pos) * 0.5f;
    Eigen::Vector3f vel_sp(result.velocity_waypoint.linear.x,
                           result.velocity_waypoint.linear.y,
                           result.velocity_waypoint.linear.z);

    float angle_pos_vel_sp = std::atan2(pos_to_pos_sp.cross(vel_sp).norm(),
                                        pos_to_pos_sp.dot(vel_sp));
    EXPECT_NEAR(0.0, angle_pos_vel_sp, 1.0);

    // calculate new vehicle position
    Eigen::Vector3f new_pos =
        pos + 1.5f * pos_to_pos_sp;  // the 1.5 coefficient makes sure to
                                     // progress trough the tree nodes
    position.pose.position = toPoint(new_pos);
  }
}
