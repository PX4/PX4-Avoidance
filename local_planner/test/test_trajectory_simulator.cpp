#include <gtest/gtest.h>
#include <limits>
#include "../include/local_planner/trajectory_simulator.h"

using namespace avoidance;

void expect_respects_limits(simulation_limits config, simulation_state start,
                            float step_time, int num_steps,
                            const std::vector<simulation_state>& steps) {
  const simulation_state* last_step = &start;
  EXPECT_EQ(num_steps, steps.size());
  for (const auto& step : steps) {
    Eigen::Vector3f jerk =
        (step.acceleration - last_step->acceleration) / step_time;
    last_step = &step;

    EXPECT_LE(jerk.norm(), config.max_jerk_norm);
    EXPECT_LE(step.acceleration.norm(), config.max_acceleration_norm);
    EXPECT_LE(step.velocity.topRows<2>().norm(), config.max_xy_velocity_norm);
  }
}

void expect_goes_in_goal_direction(Eigen::Vector3f goal_dir,
                                   simulation_state start, float step_time,
                                   const std::vector<simulation_state>& steps) {
  const simulation_state* last_step = &start;
  for (const auto& step : steps) {
    Eigen::Vector3f jerk =
        (step.acceleration - last_step->acceleration) / step_time;
    last_step = &step;

    // things to check:
    // 1) acceleration should be to correct norm(vel) != goal_vel
    // 2) if it isn't, then jerk should be trying to correct accel in goal
    // direction

    Eigen::Vector3f vel_dir_error =
        step.velocity.normalized() - goal_dir.normalized();

    if (vel_dir_error.dot(step.acceleration.normalized()) < 0) {
      EXPECT_GE(jerk.dot(goal_dir), 0);
    }
  }
}

TEST(TrajectorySimulator, normClampWorksWithZeros) {
  // GIVEN: a vector shorter than the clamp
  Eigen::Vector3f short_vec(0, 0, 0);

  // WHEN we try to clamp it
  auto clamped = norm_clamp(short_vec, 0);

  // THEN: it should be zero, not NaN
  EXPECT_TRUE(clamped.norm() == 0.f);
  EXPECT_FALSE(clamped.array().isNaN().any());
}

TEST(TrajectorySimulator, normClampPassesShortVectors) {
  // GIVEN: a vector shorter than the clamp
  Eigen::Vector3f short_vec(0.5f, 0.6f, 0.7f);

  // WHEN we try to clamp it
  auto clamped = norm_clamp(short_vec, 5.f);

  // THEN: it should match exactly the input
  EXPECT_TRUE((short_vec - clamped).norm() == 0.f);
}

TEST(TrajectorySimulator, normClampClampsLongVectors) {
  // GIVEN: a vector longer than the clamp
  Eigen::Vector3f long_vec(5.f, 6.f, 7.f);

  // WHEN we try to clamp it
  auto clamped = norm_clamp(long_vec, 5.f);

  // THEN: it should have length clamped to
  EXPECT_FLOAT_EQ(5.f, clamped.norm());

  // AND: should be in the same direction as the original
  EXPECT_FLOAT_EQ(1.f, clamped.normalized().dot(long_vec.normalized()));
}

TEST(TrajectorySimulator, givesEmptyListWithNoSteps) {
  // GIVEN: a starting point, and not set up simulation limits
  simulation_state state;
  simulation_limits config;
  Eigen::Vector3f goal_dir;
  float step_time = 0.01;

  // WHEN: we get the list of trajectories with 0 steps
  int num_steps = 0;
  std::vector<simulation_state> steps =
      run_steps(config, state, goal_dir, step_time, num_steps);

  // THEN: we should get an empty list
  EXPECT_EQ(0, steps.size());
}

TEST(TrajectorySimulator, givesConstantVelWhenVelCorrect) {
  // GIVEN: a starting point
  simulation_state state;
  state.position = Eigen::Vector3f::Zero();
  state.velocity << 3.f, 0.f, 0.f;
  state.acceleration = Eigen::Vector3f::Zero();

  simulation_limits config;
  config.max_z_velocity = 1.f;
  config.min_z_velocity = -0.5f;
  config.max_xy_velocity_norm = 3.f;
  config.max_acceleration_norm = 3.f;
  config.max_jerk_norm = 20.f;

  Eigen::Vector3f goal_dir(1.f, 0.f, 0.f);
  float step_time = 0.01;

  // WHEN: we get the list of trajectories with 0 steps
  int num_steps = 10;
  std::vector<simulation_state> steps =
      run_steps(config, state, goal_dir, step_time, num_steps);

  // THEN: we should get a list with all the same velocity, zero accel
  EXPECT_EQ(num_steps, steps.size());
  for (const auto& step : steps) {
    EXPECT_TRUE((state.velocity - step.velocity).norm() < 1e-5);
    EXPECT_TRUE(step.acceleration.norm() < 1e-5);
  }
  EXPECT_NO_FATAL_FAILURE(
      expect_respects_limits(config, state, step_time, num_steps, steps));
}

TEST(TrajectorySimulator, acceleratesToConstantVel) {
  // GIVEN: a starting point
  simulation_state state;
  state.position = Eigen::Vector3f::Zero();
  state.velocity << -3.f, 0.f, 0.f;
  state.acceleration = Eigen::Vector3f::Zero();

  simulation_limits config;
  config.max_z_velocity = 1.f;
  config.min_z_velocity = -0.5f;
  config.max_xy_velocity_norm = 3.f;
  config.max_acceleration_norm = 3.f;
  config.max_jerk_norm = 20.f;

  Eigen::Vector3f goal_dir(1.f, 0.f, 0.f);
  float step_time = 0.1;

  // WHEN: we get the list of trajectories with 0 steps
  int num_steps = 300;
  std::vector<simulation_state> steps =
      run_steps(config, state, goal_dir, step_time, num_steps);

  // THEN: we should get a list where it moves in the goal direction
  EXPECT_NO_FATAL_FAILURE(
      expect_respects_limits(config, state, step_time, num_steps, steps));
  EXPECT_NO_FATAL_FAILURE(
      expect_goes_in_goal_direction(goal_dir, state, step_time, steps));
}
