#include <gtest/gtest.h>

#include "../include/safe_landing_planner/waypoint_generator.hpp"

using namespace avoidance;

class WaypointGeneratorTests : public WaypointGenerator, public ::testing::Test {
 public:
  Eigen::Vector3f published_position = Eigen::Vector3f(NAN, NAN, NAN);
  Eigen::Vector3f published_velocity = Eigen::Vector3f(NAN, NAN, NAN);
  float published_yaw = NAN;
  float published_yaw_velocity = NAN;

  void SetUp() override {
    grid_slp_.resize(40, 1);
    pos_index_ << 30, 30;
    publishTrajectorySetpoints_ = [this](const Eigen::Vector3f &pos_sp, const Eigen::Vector3f &vel_sp, float yaw_sp,
                                         float yaw_speed_sp) {
      published_position = pos_sp;
      published_velocity = vel_sp;
      published_yaw = yaw_sp;
      published_yaw_velocity = yaw_speed_sp;
      smoothing_land_cell_ = 6;
    };
  }
};

TEST_F(WaypointGeneratorTests, stateMachineStartsCorrectly) {
  // GIVEN: a basic waypoint generator (this)

  // THEN: the starting state should be goTo
  ASSERT_EQ(SLPState::GOTO, getState());
}

TEST_F(WaypointGeneratorTests, stateMachineRunsOnce) {
  // GIVEN: a basic waypoint generator (this)
  ASSERT_EQ(SLPState::GOTO, getState());

  // WHEN: we get a waypoint
  calculateWaypoint();

  // THEN: the state should still be goTo
  ASSERT_EQ(SLPState::GOTO, getState());
}

TEST_F(WaypointGeneratorTests, goTo_to_altitudechange) {
  // GIVEN: a basic waypoint generator
  ASSERT_EQ(SLPState::GOTO, getState());

  // WHEN: we get above our landing location, but altitude is wrong
  goal_ << 10, 10, 0;
  position_ << 10, 10, 50;
  is_land_waypoint_ = true;

  calculateWaypoint();

  // THEN: the state should be altitudeChange
  ASSERT_EQ(SLPState::ALTITUDE_CHANGE, getState());

  // AND: there should be a reasonable published position/velocity
  ASSERT_EQ((published_position - goal_).norm(), 0);
  ASSERT_TRUE(published_velocity.array().isNaN().all());
}

TEST_F(WaypointGeneratorTests, goTo_to_loiter) {
  // GIVEN: a basic waypoint generator
  ASSERT_EQ(SLPState::GOTO, getState());

  // WHEN: we get above our landing location, with a correct altitude
  goal_ << 10, 10, 0;
  position_ << 10, 10, 4.5;
  is_land_waypoint_ = true;

  calculateWaypoint();

  // THEN: the state should be loiter
  ASSERT_EQ(SLPState::LOITER, getState());

  // AND: there should be a reasonable published position/velocity
  ASSERT_EQ((published_position - goal_).norm(), 0);
  ASSERT_TRUE(published_velocity.array().isNaN().all());
}

TEST_F(WaypointGeneratorTests, altitudechange_transitions) {
  // GIVEN: a basic waypoint generator, that switched to altitude change after
  // first iteration
  ASSERT_EQ(SLPState::GOTO, getState());

  goal_ << 10, 10, 0;
  position_ << 10, 10, 50;
  is_land_waypoint_ = true;

  calculateWaypoint();
  ASSERT_EQ(SLPState::ALTITUDE_CHANGE, getState());

  // WHEN: we are above our landing location, but altitude is wrong
  for (int i = 0; i < 10; i++) {
    calculateWaypoint();
    // THEN: the state should remain altitudeChange
    ASSERT_EQ(SLPState::ALTITUDE_CHANGE, getState());

    // AND: published position should be (x, y, nan) and velocity (nan, nan, z)
    ASSERT_TRUE(std::isnan(published_position.z()));
    ASSERT_EQ(Eigen::Vector2f(published_position.x() - goal_.x(), published_position.y() - goal_.y()).norm(), 0);
    ASSERT_TRUE(std::isnan(published_velocity.x()));
    ASSERT_TRUE(std::isnan(published_velocity.y()));
    ASSERT_FALSE(std::isnan(published_velocity.z()));
  }

  // WHEN: we are above our landing location at the correct altitude
  position_ << 10, 10, 4.5;
  calculateWaypoint();

  // THEN: the state should remain altitudeChange
  ASSERT_EQ(SLPState::LOITER, getState());

  // AND: published position should be (x, y, nan) and velocity (nan, nan, z)
  ASSERT_TRUE(std::isnan(published_position.z()));
  ASSERT_EQ(Eigen::Vector2f(published_position.x() - goal_.x(), published_position.y() - goal_.y()).norm(), 0);
  ASSERT_TRUE(std::isnan(published_velocity.x()));
  ASSERT_TRUE(std::isnan(published_velocity.y()));
  ASSERT_FALSE(std::isnan(published_velocity.z()));
}

TEST_F(WaypointGeneratorTests, loiter_to_land) {
  // GIVEN: a basic waypoint generator,  that switched to loiter after first
  // iteration
  ASSERT_EQ(SLPState::GOTO, getState());

  goal_ << 10, 10, 0;
  position_ << 10, 10, 4.5;
  is_land_waypoint_ = true;
  can_land_ = 0;

  calculateWaypoint();
  ASSERT_EQ(SLPState::LOITER, getState());

  // WHEN: we are loitering above our landing location
  for (int i = 0; i < 10; i++) {
    calculateWaypoint();
    // THEN: the state should remain loiter, the position setpoint should
    // correspond to current position and the velocity setpoint should be NAN
    ASSERT_EQ(SLPState::LOITER, getState());
    ASSERT_EQ((published_position - position_).norm(), 0);
    ASSERT_TRUE(published_velocity.array().isNaN().all());
  }

  // WHEN: the data shows that landing is possible
  grid_slp_seq_ = 25;
  can_land_hysteresis_matrix_.fill(can_land_thr_ + 1);
  calculateWaypoint();

  // THEN: the state should switch to land, the position setpoint should
  // correspond to current position and the velocity setpoint should be NAN
  ASSERT_EQ(SLPState::LAND, getState());
  ASSERT_EQ((published_position - position_).norm(), 0);
  ASSERT_TRUE(published_velocity.array().isNaN().all());
}

TEST_F(WaypointGeneratorTests, loiter_to_goTo) {
  // GIVEN: a basic waypoint generator,  that switched to loiter after the grid exploration
  ASSERT_EQ(SLPState::GOTO, getState());

  goal_ << 10, 10, 0;
  position_ << 10, 10, 4.5;
  is_land_waypoint_ = true;
  start_grid_exploration_ = false;
  can_land_ = 0;

  calculateWaypoint();
  ASSERT_EQ(SLPState::LOITER, getState());

  // WHEN: we are loitering above our landing location
  for (int i = 0; i < 10; i++) {
    calculateWaypoint();
    // THEN: the state should remain loiter, the position setpoint should
    // correspond to current position and the velocity setpoint should be NAN
    ASSERT_EQ(SLPState::LOITER, getState());
    ASSERT_EQ((published_position - position_).norm(), 0);
    ASSERT_TRUE(published_velocity.array().isNaN().all());
  }

  // WHEN: the data allows no landing
  grid_slp_seq_ = 25;
  calculateWaypoint();

  // THEN: the state should switch to GOTO to do the spiral pattern, the position setpoint should
  // correspond to current position and the velocity setpoint should be NAN
  ASSERT_EQ(SLPState::GOTO, getState());
  ASSERT_EQ((published_position - position_).norm(), 0);
  ASSERT_TRUE(published_velocity.array().isNaN().all());
}

TEST_F(WaypointGeneratorTests, loiter_to_evaluateGrid_to_Land) {
  // GIVEN: a basic waypoint generator,  that switched to loiter after first
  // iteration
  ASSERT_EQ(SLPState::GOTO, getState());

  goal_ << 10, 10, 0;
  position_ << 10, 10, 4.5;
  is_land_waypoint_ = true;
  start_grid_exploration_ = true;
  can_land_ = 0;

  calculateWaypoint();
  ASSERT_EQ(SLPState::LOITER, getState());
  can_land_hysteresis_matrix_ << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1,
      1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1,
      1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
      1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1,
      1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1,
      1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1,
      1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

  // WHEN: the data allows no landing, first do a grid exploration
  grid_slp_seq_ = 25;

  calculateWaypoint();

  // THEN: the state should switch to EVALUATE_GRID, the position setpoint should
  // correspond to current position and the velocity setpoint should be NAN
  ASSERT_EQ(SLPState::EVALUATE_GRID, getState());
  ASSERT_EQ((published_position - position_).norm(), 0);
  ASSERT_TRUE(published_velocity.array().isNaN().all());

  // THEN: grid exploration allows landing, go to goto
  calculateWaypoint();
  ASSERT_EQ(SLPState::GOTO, getState());

  calculateWaypoint();
  ASSERT_EQ(SLPState::GOTO, getState());

  // THEN: reached the founded the landing spot, go to LAND state
  position_ << 22, 22, 4.5;
  calculateWaypoint();

  ASSERT_EQ(SLPState::LAND, getState());
}

TEST_F(WaypointGeneratorTests, loiter_to_evaluateGrid) {
  // GIVEN: a basic waypoint generator,  that switched to loiter after first
  // iteration
  ASSERT_EQ(SLPState::GOTO, getState());

  goal_ << 10, 10, 0;
  position_ << 10, 10, 4.5;
  is_land_waypoint_ = true;
  start_grid_exploration_ = true;
  can_land_ = 0;

  calculateWaypoint();
  ASSERT_EQ(SLPState::LOITER, getState());

  // WHEN: the data allows no landing, first do a grid exploration
  grid_slp_seq_ = 25;

  calculateWaypoint();

  // THEN: the state should switch to EVALUATE_GRID, the position setpoint should
  // correspond to current position and the velocity setpoint should be NAN
  ASSERT_EQ(SLPState::EVALUATE_GRID, getState());
  ASSERT_EQ((published_position - position_).norm(), 0);
  ASSERT_TRUE(published_velocity.array().isNaN().all());

  // THEN: grid exploration allows no landing, go to loiter
  calculateWaypoint();
  ASSERT_EQ(SLPState::LOITER, getState());
}

TEST_F(WaypointGeneratorTests, land_transitions) {
  // GIVEN: a basic waypoint generator,  that switched to from goto -> loiter ->
  // land
  ASSERT_EQ(SLPState::GOTO, getState());

  goal_ << 10, 10, 0;
  position_ << 10, 10, 4.5;
  is_land_waypoint_ = true;
  can_land_ = 0;

  calculateWaypoint();
  ASSERT_EQ(SLPState::LOITER, getState());

  grid_slp_seq_ = 25;
  can_land_hysteresis_matrix_.fill(can_land_thr_ + 1);

  calculateWaypoint();

  ASSERT_EQ(SLPState::LAND, getState());

  // WHEN: we calculate the next waypoint without changing the data
  for (int i = 0; i < 10; i++) {
    calculateWaypoint();

    // THEN: the state should remain land
    ASSERT_EQ(SLPState::LAND, getState());
    // AND: published position should be (x, y, nan) and velocity (nan, nan, z)
    ASSERT_TRUE(std::isnan(published_position.z()));
    ASSERT_EQ(Eigen::Vector2f(published_position.x() - goal_.x(), published_position.y() - goal_.y()).norm(), 0);
    ASSERT_TRUE(std::isnan(published_velocity.x()));
    ASSERT_TRUE(std::isnan(published_velocity.y()));
    ASSERT_FALSE(std::isnan(published_velocity.z()));
  }

  // WHEN: an error is triggered
  trigger_reset_ = true;
  calculateWaypoint();
  ASSERT_EQ(SLPState::GOTO, getState());
  // AND: published position should be (x, y, nan) and velocity (nan, nan, z)
  ASSERT_TRUE(std::isnan(published_position.z()));
  ASSERT_EQ(Eigen::Vector2f(published_position.x() - goal_.x(), published_position.y() - goal_.y()).norm(), 0);
  ASSERT_TRUE(std::isnan(published_velocity.x()));
  ASSERT_TRUE(std::isnan(published_velocity.y()));
  ASSERT_FALSE(std::isnan(published_velocity.z()));
}
