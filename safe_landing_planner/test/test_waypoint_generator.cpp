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
      px4_.param_mpc_land_speed = 0.7f;
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

TEST_F(WaypointGeneratorTests, altitudeChange_to_Loiter) {
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

TEST_F(WaypointGeneratorTests, loiter_to_evaluateGrid) {
  // GIVEN: a basic waypoint generator, that switched to altitude change after
  // first iteration
  ASSERT_EQ(SLPState::GOTO, getState());

  // WHEN: we are above our landing location
  goal_ << 10, 10, 0;
  position_ << 10, 10, 8;
  is_land_waypoint_ = true;

  // THEN: the state should go to altitudeChange
  calculateWaypoint();
  ASSERT_EQ(SLPState::ALTITUDE_CHANGE, getState());

  // WHEN: we are above our landing location and the altitude is correct
  position_ << 10, 10, 4.2;
  calculateWaypoint();
  // THEN: the state should go to Loiter
  ASSERT_EQ(SLPState::LOITER, getState());

  // WHEN: data is ready
  grid_slp_seq_ = 25;
  calculateWaypoint();
  // THEN: the state should go to EvaluateGrid
  ASSERT_EQ(SLPState::EVALUATE_GRID, getState());
  // AND: published position should be loiter_position is x, y, z and velocity (nan, nan, nan)
  ASSERT_TRUE(published_velocity.array().isNaN().all());
  ASSERT_EQ(Eigen::Vector2f(published_position.x() - position_.x(), published_position.y() - position_.y()).norm(), 0);
}

TEST_F(WaypointGeneratorTests, evaluateGrid_to_goToLand) {
  // GIVEN: a basic waypoint generator, that switched to altitude change after
  // first iteration
  ASSERT_EQ(SLPState::GOTO, getState());

  // WHEN: we are above our landing location and at the correct altitude
  goal_ << 10, 10, 0;
  position_ << 10, 10, 4.5;
  is_land_waypoint_ = true;

  // THEN: the state should go to altitudeChange
  calculateWaypoint();
  ASSERT_EQ(SLPState::ALTITUDE_CHANGE, getState());

  // THEN: the state should go to Loiter
  calculateWaypoint();
  ASSERT_EQ(SLPState::LOITER, getState());

  // WHEN: data is ready
  grid_slp_seq_ = 25;
  calculateWaypoint();
  // THEN: the state should go to EvaluateGrid
  ASSERT_EQ(SLPState::EVALUATE_GRID, getState());

  // WHEN: a landing area is available in the current grid
  can_land_hysteresis_result_.fill(1);
  calculateWaypoint();
  // THEN: the state should go to goto_land
  ASSERT_EQ(SLPState::GOTO_LAND, getState());
  // AND: published position should be loiter_position is x, y, z and velocity (nan, nan, nan)
  ASSERT_TRUE(published_velocity.array().isNaN().all());
  ASSERT_EQ(Eigen::Vector2f(published_position.x() - position_.x(), published_position.y() - position_.y()).norm(), 0);
}

TEST_F(WaypointGeneratorTests, evaluateGrid_to_goTo) {
  // GIVEN: a basic waypoint generator, that switched to altitude change after
  // first iteration
  ASSERT_EQ(SLPState::GOTO, getState());

  // WHEN: we are above our landing location and at the correct altitude
  goal_ << 10, 10, 0;
  position_ << 10, 10, 4.5;
  is_land_waypoint_ = true;

  // THEN: the state should go to altitudeChange
  calculateWaypoint();
  ASSERT_EQ(SLPState::ALTITUDE_CHANGE, getState());

  // THEN: the state should go to Loiter
  calculateWaypoint();
  ASSERT_EQ(SLPState::LOITER, getState());

  // WHEN: data is ready and there isn't any landing area in the current grid
  grid_slp_seq_ = 25;
  can_land_hysteresis_matrix_.fill(can_land_thr_ - 0.2);
  calculateWaypoint();
  // THEN: the state should go to EvaluateGrid
  ASSERT_EQ(SLPState::EVALUATE_GRID, getState());

  calculateWaypoint();
  // THEN: the state should go to goto
  ASSERT_EQ(SLPState::GOTO, getState());
  // AND: published position should be loiter_position is x, y, z and velocity (nan, nan, nan)
  ASSERT_TRUE(published_velocity.array().isNaN().all());
  ASSERT_EQ(Eigen::Vector2f(published_position.x() - position_.x(), published_position.y() - position_.y()).norm(), 0);
}

TEST_F(WaypointGeneratorTests, land_transitions) {
  // GIVEN: a basic waypoint generator,  that switched to from goto -> loiter ->
  // land
  ASSERT_EQ(SLPState::GOTO, getState());

  // WHEN: we are above our landing location and at the correct altitude
  goal_ << 10, 10, 0;
  position_ << 10, 10, 4.5;
  is_land_waypoint_ = true;
  can_land_ = 0;

  // THEN: the state should go to altitudeChange
  calculateWaypoint();
  ASSERT_EQ(SLPState::ALTITUDE_CHANGE, getState());

  // THEN: the state should go to Loiter
  calculateWaypoint();
  ASSERT_EQ(SLPState::LOITER, getState());

  // WHEN: data is ready and there is a landing area in the current grid
  grid_slp_seq_ = 25;
  can_land_hysteresis_matrix_.fill(can_land_thr_ + 1);
  calculateWaypoint();
  // THEN: the state should go to EvaluateGrid
  ASSERT_EQ(SLPState::EVALUATE_GRID, getState());

  calculateWaypoint();
  // THEN: the state should go to goto_land
  ASSERT_EQ(SLPState::GOTO_LAND, getState());

  // WHEN: we are above the landing location
  calculateWaypoint();
  // THEN: the state should go to land
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
