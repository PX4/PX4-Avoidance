#include <gtest/gtest.h>

#include "../include/safe_landing_planner/waypoint_generator.hpp"

using namespace avoidance;

class WaypointGeneratorTests : public WaypointGenerator,
                               public ::testing::Test {
public:
  Eigen::Vector3f published_position = Eigen::Vector3f(NAN, NAN, NAN);
  Eigen::Vector3f published_velocity = Eigen::Vector3f(NAN, NAN, NAN);
  float published_yaw = NAN;
  float published_yaw_velocity = NAN;

  void SetUp() override {
    grid_slp_.resize(40, 1);
    pos_index_ << 30, 30;
    publishTrajectorySetpoints_ = [this](const Eigen::Vector3f &pos_sp,
                                         const Eigen::Vector3f &vel_sp,
                                         float yaw_sp, float yaw_speed_sp) {
      published_position = pos_sp;
      published_velocity = vel_sp;
      published_yaw = yaw_sp;
      published_yaw_velocity = yaw_speed_sp;
    };
  }
};

TEST_F(WaypointGeneratorTests, stateMachineStartsCorrectly) {
  // GIVEN: a basic waypoint generator (this)

  // THEN: the starting state should be goTo
  ASSERT_EQ(SLPState::goTo, getState());
}

TEST_F(WaypointGeneratorTests, stateMachineRunsOnce) {
  // GIVEN: a basic waypoint generator (this)
  ASSERT_EQ(SLPState::goTo, getState());

  // WHEN: we get a waypoint
  calculateWaypoint();

  // THEN: the state should still be goTo
  ASSERT_EQ(SLPState::goTo, getState());
}

TEST_F(WaypointGeneratorTests, goto_to_altitudechange) {
  // GIVEN: a basic waypoint generator
  ASSERT_EQ(SLPState::goTo, getState());

  // WHEN: we get above our landing location, but altitude is wrong
  goal_ << 10, 10, 0;
  position_ << 10, 10, 50;
  is_land_waypoint_ = true;

  calculateWaypoint();

  // THEN: the state should be altitudeChange
  ASSERT_EQ(SLPState::altitudeChange, getState());

  // AND: there should be a reasonable published position/velocity
  ASSERT_EQ((published_position - goal_).norm(), 0);
  ASSERT_TRUE(published_velocity.array().isNaN().all());

}
