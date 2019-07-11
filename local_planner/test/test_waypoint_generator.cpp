#include <gtest/gtest.h>

#include "../include/local_planner/waypoint_generator.h"
#include "avoidance/common.h"

using namespace avoidance;

class WaypointGeneratorTests : public ::testing::Test, public WaypointGenerator {
 public:
  avoidanceOutput avoidance_output;
  Eigen::Vector3f position;
  Eigen::Quaternionf q;
  Eigen::Vector3f goal;
  Eigen::Vector3f prev_goal;
  Eigen::Vector3f velocity;
  NavigationState nav_state;
  bool is_land_waypoint;
  bool is_takeoff_waypoint;
  Eigen::Vector3f desired_velocity;

  bool stay = false;
  bool is_airborne = true;
  ros::Time time = ros::Time(0.33);

  ros::Time getSystemTime() override { return time; }

  void SetUp() override {
    ros::Time::init();

    avoidance_output.waypoint_type = direct;
    avoidance_output.obstacle_ahead = false;
    avoidance_output.cruise_velocity = 1.0;
    avoidance_output.last_path_time = ros::Time(0.28);

    PolarPoint p_pol = histogramIndexToPolar(15, 35, 6, 0.f);

    float n1_x = 0.8f;
    float n2_x = 1.3f;
    float n3_x = 2.2f;
    float n4_x = 2.9f;
    float n5_x = 3.5f;
    Eigen::Vector3f n0(0.0f, 0.0f, 2.0f);
    Eigen::Vector3f n1(n1_x, n0.y() + sqrtf(1 - (n1_x * n1_x)), 2.0f);
    Eigen::Vector3f n2(n2_x, n1.y() + sqrtf(1 - powf(n2_x - n1.x(), 2)), 2.0f);
    Eigen::Vector3f n3(n3_x, n2.y() + sqrtf(1 - powf(n3_x - n2.x(), 2)), 2.0f);
    Eigen::Vector3f n4(n4_x, n3.y() + sqrtf(1 - powf(n4_x - n3.x(), 2)), 2.0f);
    Eigen::Vector3f n5(n5_x, n4.y() + sqrtf(1 - powf(n5_x - n4.x(), 2)), 2.0f);
    avoidance_output.path_node_positions = {n5, n4, n3, n2, n1, n0};

    position = Eigen::Vector3f(0.f, 0.f, 0.f);
    q = Eigen::Quaternionf(1.f, 0.f, 0.f, 0.f);
    goal = Eigen::Vector3f(10.f, 3.f, 2.f);
    velocity = Eigen::Vector3f(0.f, 0.f, 0.f);
    prev_goal = Eigen::Vector3f(0.f, 0.f, 2.f);
    nav_state = NavigationState::mission;
    is_land_waypoint = false;
    is_takeoff_waypoint = false;
    desired_velocity = Eigen::Vector3f(NAN, NAN, NAN);

    updateState(position, q, goal, prev_goal, velocity, stay, is_airborne, nav_state, is_land_waypoint,
                is_takeoff_waypoint, desired_velocity);
    setPlannerInfo(avoidance_output);
    FOV fov(0.0f, 0.0f, 270.f, 45.f);
    setFOV(0, fov);
  }
  void TearDown() override {}
};

TEST_F(WaypointGeneratorTests, reachAltitudeTest) {
  // GIVEN: a waypoint of type goFast and the vehicle has not yet reached the
  // goal altiude
  avoidance_output.waypoint_type = reachHeight;
  goal << 0.f, 0.f, 5.f;
  setPlannerInfo(avoidance_output);
  double time_sec = 0.0;
  float adapted_to_goal_prev = 1000.0f;
  float pos_sp_to_goal_prev = 1000.0f;
  is_takeoff_waypoint = true;
  desired_velocity.z() = 1.5f;

  // WHEN: we generate the first waypoint
  time = ros::Time(time_sec);
  updateState(position, q, goal, prev_goal, velocity, stay, is_airborne, nav_state, is_land_waypoint,
              is_takeoff_waypoint, desired_velocity);
  waypointResult result = getWaypoints();

  // THEN: we expect the goto position to point straight up
  EXPECT_NEAR(goal.x(), result.goto_position.x(), 0.1);
  EXPECT_NEAR(goal.y(), result.goto_position.y(), 0.1);
  EXPECT_LT(position.z(), result.goto_position.z());

  // THEN: we expect the adapted goto position to be between goal and drone in z
  EXPECT_GT(result.adapted_goto_position.z(), position.z());
  EXPECT_LT(result.adapted_goto_position.z(), goal.z());

  // THEN: we expect the adapted goto position to be close to the drone location
  // in xy
  EXPECT_NEAR(position.x(), result.adapted_goto_position.x(), 0.1);
  EXPECT_NEAR(position.y(), result.adapted_goto_position.y(), 0.1);

  // THEN: we expect the smoothed goto position to be the same as the drone
  // location
  // (first iteration of smoothing)
  EXPECT_NEAR(position.x(), result.smoothed_goto_position.x(), 0.1);
  EXPECT_NEAR(position.y(), result.smoothed_goto_position.y(), 0.1);
  EXPECT_NEAR(position.z(), result.smoothed_goto_position.z(), 0.1);

  // THEN: we expect the smoothed goto position to be the position waypoint,
  // since smoothing was enabled
  EXPECT_EQ(result.smoothed_goto_position.x(), result.position_wp.x());
  EXPECT_EQ(result.smoothed_goto_position.y(), result.position_wp.y());
  EXPECT_EQ(result.smoothed_goto_position.z(), result.position_wp.z());

  ASSERT_TRUE(std::isfinite(result.linear_velocity_wp.z()));

  // WHEN: we generate subsequent waypoints
  for (size_t i = 0; i < 10; i++) {
    // calculate new vehicle position
    time_sec += 0.03;
    time = ros::Time(time_sec);
    updateState(position, q, goal, prev_goal, velocity, stay, is_airborne, nav_state, is_land_waypoint,
                is_takeoff_waypoint, desired_velocity);
    waypointResult result = getWaypoints();

    // THEN: we expect the goto location to point straight up
    EXPECT_NEAR(position.x(), result.goto_position.x(), 0.1);
    EXPECT_NEAR(position.y(), result.goto_position.y(), 0.1);
    EXPECT_LT(position.z(), result.goto_position.z());

    // THEN: we expect the adapted goto position to be between goal and drone in
    // z
    EXPECT_GT(result.adapted_goto_position.z(), position.z());
    EXPECT_LT(result.adapted_goto_position.z(), goal.z());

    // THEN: we expect the adapted goto position to be close to the drone
    // location in xy
    EXPECT_NEAR(position.x(), result.adapted_goto_position.x(), 0.1);
    EXPECT_NEAR(position.y(), result.adapted_goto_position.y(), 0.1);

    // THEN: we expect the smoothed goto position to be the position waypoint,
    // since smoothing was enabled
    EXPECT_LT(position.z(), result.smoothed_goto_position.z());
    EXPECT_GT(result.adapted_goto_position.z(), result.smoothed_goto_position.z());

    // THEN: we expect a finite z velocity component on the setpoint
    ASSERT_TRUE(std::isfinite(result.linear_velocity_wp.z()));

    // THEN: we expect the z component of the waypoint to move closer to goal.z
    float adapted_to_goal = std::abs(goal.z() - result.adapted_goto_position.z());
    float pos_sp_to_goal = std::abs(goal.z() - result.position_wp.z());
    ASSERT_LT(adapted_to_goal, adapted_to_goal_prev);
    ASSERT_LT(pos_sp_to_goal, pos_sp_to_goal_prev);
    adapted_to_goal_prev = adapted_to_goal;
    pos_sp_to_goal_prev = pos_sp_to_goal;

    // Update the state for next iteration, assume we get half-way from current
    // location toward the position setpoint
    Eigen::Vector3f pos_to_pos_sp = (result.position_wp - position) * 0.5f;
    Eigen::Vector3f new_pos = position + pos_to_pos_sp;
    position = new_pos;
  }
}

TEST_F(WaypointGeneratorTests, goStraightTest) {
  // GIVEN: a waypoint of type goStraight
  avoidance_output.waypoint_type = direct;
  is_takeoff_waypoint = false;
  desired_velocity.z() = NAN;
  setPlannerInfo(avoidance_output);

  float goto_to_goal_prev = 1000.0f;
  float adapted_to_goal_prev = 1000.0f;
  float pos_sp_to_goal_prev = 1000.0f;
  double time_sec = time.toSec();

  // WHEN: we generate waypoints
  for (size_t i = 0; i < 10; i++) {
    time_sec += 0.03;
    time = ros::Time(time_sec);
    updateState(position, q, goal, prev_goal, velocity, stay, is_airborne, nav_state, is_land_waypoint,
                is_takeoff_waypoint, desired_velocity);

    waypointResult result = getWaypoints();
    float goto_to_goal = (goal - result.goto_position).norm();
    float adapted_to_goal = (goal - result.adapted_goto_position).norm();
    float pos_sp_to_goal = (goal - result.position_wp).norm();
    // THEN: we expect the waypoints to move closer to the goal
    ASSERT_LE(goto_to_goal, goto_to_goal_prev);
    ASSERT_LE(adapted_to_goal, adapted_to_goal_prev);
    ASSERT_LE(pos_sp_to_goal, pos_sp_to_goal_prev);
    goto_to_goal_prev = goto_to_goal;
    adapted_to_goal_prev = adapted_to_goal;
    pos_sp_to_goal_prev = pos_sp_to_goal;

    // Assume we get halfway from current position to the setpoint
    Eigen::Vector3f pos_to_pos_sp = (result.position_wp - position) * 0.5;

    // THEN: we expect NAN velocity setpoints
    ASSERT_FALSE(std::isfinite(result.linear_velocity_wp.x()));
    ASSERT_FALSE(std::isfinite(result.linear_velocity_wp.y()));
    ASSERT_FALSE(std::isfinite(result.linear_velocity_wp.z()));

    // calculate new vehicle position
    Eigen::Vector3f new_pos = position + pos_to_pos_sp;
    position = new_pos;
  }
}

TEST_F(WaypointGeneratorTests, hoverTest) {
  // GIVEN: a waypoint of type hover

  // first run one the waypoint generator such that smoothed_goto_location_ gets
  // initialize
  setPlannerInfo(avoidance_output);
  double time_sec = 0.0;
  time = ros::Time(time_sec);
  updateState(position, q, goal, prev_goal, velocity, stay, is_airborne, nav_state, is_land_waypoint,
              is_takeoff_waypoint, desired_velocity);
  waypointResult result = getWaypoints();

  avoidance_output.waypoint_type = hover;
  setPlannerInfo(avoidance_output);
  time_sec += 0.033;
  time = ros::Time(time_sec);
  updateState(position, q, goal, prev_goal, velocity, stay, is_airborne, nav_state, is_land_waypoint,
              is_takeoff_waypoint, desired_velocity);

  // WHEN: we generate waypoints
  result = getWaypoints();

  // THEN: we expect the position waypoint to be the same as the current vehicle
  // position
  EXPECT_NEAR(position.x(), result.goto_position.x(), 0.001);
  EXPECT_NEAR(position.y(), result.goto_position.y(), 0.001);
  EXPECT_NEAR(position.z(), result.goto_position.z(), 0.001);

  EXPECT_NEAR(position.x(), result.position_wp.x(), 0.01);
  EXPECT_NEAR(position.y(), result.position_wp.y(), 0.01);
  EXPECT_NEAR(position.z(), result.position_wp.z(), 0.01);

  EXPECT_NEAR(0.0, result.orientation_wp.x(), 0.1);
  EXPECT_NEAR(0.0, result.orientation_wp.y(), 0.1);
  EXPECT_NEAR(0.0, result.orientation_wp.z(), 0.1);
  EXPECT_NEAR(1.0, result.orientation_wp.w(), 0.1);

  // THEN: we expect NAN velocity setpoints
  ASSERT_FALSE(std::isfinite(result.linear_velocity_wp.x()));
  ASSERT_FALSE(std::isfinite(result.linear_velocity_wp.y()));
  ASSERT_FALSE(std::isfinite(result.linear_velocity_wp.z()));
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
    updateState(position, q, goal, prev_goal, velocity, stay, is_airborne, nav_state, is_land_waypoint,
                is_takeoff_waypoint, desired_velocity);

    waypointResult result = getWaypoints();
    float goto_to_goal = (goal - result.goto_position).norm();
    float adapted_to_goal = (goal - result.adapted_goto_position).norm();
    float pos_sp_to_goal = (goal - result.position_wp).norm();

    ASSERT_LE(goto_to_goal, goto_to_goal_prev);
    ASSERT_LE(adapted_to_goal, adapted_to_goal_prev);
    ASSERT_LE(pos_sp_to_goal, pos_sp_to_goal_prev);
    goto_to_goal_prev = goto_to_goal;
    adapted_to_goal_prev = adapted_to_goal;
    pos_sp_to_goal_prev = pos_sp_to_goal;

    // Assume we get halfway from current position to the setpoint
    Eigen::Vector3f pos_to_pos_sp = (result.position_wp - position) * 0.5f;

    // THEN: we expect NAN velocity setpoints
    ASSERT_FALSE(std::isfinite(result.linear_velocity_wp.x()));
    ASSERT_FALSE(std::isfinite(result.linear_velocity_wp.y()));
    ASSERT_FALSE(std::isfinite(result.linear_velocity_wp.z()));

    // calculate new vehicle position
    Eigen::Vector3f new_pos = position + 1.5f * pos_to_pos_sp;  // the 1.5 coefficient makes sure to
                                                                // progress trough the tree nodes
    position = new_pos;
  }
}
