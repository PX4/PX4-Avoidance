#include <gtest/gtest.h>
#include <limits>
#include "../include/local_planner/trajectory_simulator.h"

using namespace avoidance;

void expect_respects_limits(simulation_limits config, simulation_state start,
                            const std::vector<simulation_state>& steps) {
  const simulation_state* last_step = &start;
  for (const auto& step : steps) {
    float step_time = step.time - last_step->time;
    Eigen::Vector3f jerk = (step.acceleration - last_step->acceleration) / step_time;
    last_step = &step;

    const float eps = 1e-5f;
    EXPECT_LE(jerk.norm(), config.max_jerk_norm + eps) << " step time: " << step_time;
    EXPECT_LE(step.acceleration.norm(), config.max_acceleration_norm + eps);
    EXPECT_LE(step.velocity.topRows<2>().norm(), config.max_xy_velocity_norm + eps);
  }
}

void expect_goes_in_goal_direction(Eigen::Vector3f goal_dir, simulation_state start,
                                   const std::vector<simulation_state>& steps) {
  const simulation_state* last_step = &start;
  for (const auto& step : steps) {
    float step_time = step.time - last_step->time;
    Eigen::Vector3f jerk = (step.acceleration - last_step->acceleration) / step_time;
    last_step = &step;

    // things to check:
    // 1) acceleration should be to correct norm(vel) != goal_vel
    // 2) if it isn't, then jerk should be trying to correct accel in goal
    // direction

    Eigen::Vector3f vel_dir_error = step.velocity.normalized() - goal_dir.normalized();

    if (vel_dir_error.dot(step.acceleration.normalized()) > 0) {
      EXPECT_LE(jerk.dot(vel_dir_error), 0) << "vel err:" << vel_dir_error.transpose()
                                            << "  accel: " << step.acceleration.transpose()
                                            << "  jerk:" << jerk.transpose();
    }
  }
}

void print_states(simulation_state start, const std::vector<simulation_state>& steps) {
  std::cout << std::endl;
  const simulation_state* last_step = &start;
  for (const auto& step : steps) {
    float step_time = step.time - last_step->time;
    Eigen::Vector3f jerk = (step.acceleration - last_step->acceleration) / step_time;
    last_step = &step;

    std::cout << "jer: " << jerk.transpose();
    std::cout << "    acc: " << step.acceleration.transpose();
    std::cout << "    vel: " << step.velocity.transpose();
    std::cout << "    pos: " << step.position.transpose();
    std::cout << "    time: " << step.time << std::endl;
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

  TrajectorySimulator sim(config, state);

  // WHEN: we get the list of trajectories with 0 steps
  Eigen::Vector3f goal_dir = Eigen::Vector3f::Zero();
  std::vector<simulation_state> steps = sim.generate_trajectory(goal_dir, 0);

  // THEN: we should get an empty list
  EXPECT_EQ(0, steps.size());
}

TEST(TrajectorySimulator, givesConstantVelWhenVelCorrect) {
  // GIVEN: a starting point
  simulation_state state;
  state.position = Eigen::Vector3f::Zero();
  state.velocity << 3.f, 0.f, 0.f;
  state.acceleration = Eigen::Vector3f::Zero();
  state.time = 0.f;

  simulation_limits config;
  config.max_z_velocity = 1.f;
  config.min_z_velocity = -0.5f;
  config.max_xy_velocity_norm = 3.f;
  config.max_acceleration_norm = 3.f;
  config.max_jerk_norm = 20.f;

  TrajectorySimulator sim(config, state);

  Eigen::Vector3f goal_dir(1.f, 0.f, 0.f);
  float sim_time = 10;

  // WHEN: we get the list of trajectories with 0 steps
  std::vector<simulation_state> steps = sim.generate_trajectory(goal_dir, sim_time);

  // THEN: we should get a list with all the same velocity, zero accel

  for (const auto& step : steps) {
    EXPECT_TRUE((state.velocity - step.velocity).norm() < 1e-5);
    EXPECT_TRUE(step.acceleration.norm() < 1e-5);
  }
  EXPECT_NO_FATAL_FAILURE(expect_respects_limits(config, state, steps));

  // AND: it should last the right amount of time
  simulation_state last = steps[steps.size() - 1];
  EXPECT_GT(last.time, sim_time + state.time);
}

TEST(TrajectorySimulator, acceleratesToConstantVel) {
  // GIVEN: a starting point
  simulation_state state;
  state.position = Eigen::Vector3f::Zero();
  state.velocity << -3.f, 0.f, 0.f;
  state.acceleration = Eigen::Vector3f::Zero();
  state.time = 0.f;

  simulation_limits config;
  config.max_z_velocity = 1.f;
  config.min_z_velocity = -0.5f;
  config.max_xy_velocity_norm = 3.f;
  config.max_acceleration_norm = 4.f;
  config.max_jerk_norm = 20.f;

  TrajectorySimulator sim(config, state);

  // WHEN: we get the list of trajectories with 0 steps
  Eigen::Vector3f goal_dir(1.f, 0.f, 0.f);
  float sim_time = 10;
  std::vector<simulation_state> steps = sim.generate_trajectory(goal_dir, sim_time);

  // THEN: we should get a list where it moves in the goal direction
  EXPECT_NO_FATAL_FAILURE(expect_goes_in_goal_direction(goal_dir, state, steps));

  // AND: it should respect the configured limits
  EXPECT_NO_FATAL_FAILURE(expect_respects_limits(config, state, steps));

  // AND: it should, at the end, have reached the goal velocity
  simulation_state last = steps[steps.size() - 1];
  EXPECT_LT((last.velocity.normalized() - goal_dir.normalized()).norm(), 1e-5);
  EXPECT_LT(last.acceleration.norm(), 1e-5);
  EXPECT_GT(last.time, sim_time + state.time);

  //   print_states(state, steps);
}

TEST(TrajectorySimulator, acceleratesSidewaysToConstantVel) {
  // GIVEN: a starting point
  simulation_state state;
  state.position = Eigen::Vector3f::Zero();
  state.velocity << 3.f, 0.f, 0.f;
  state.acceleration = Eigen::Vector3f::Zero();
  state.time = 8.f;

  simulation_limits config;
  config.max_z_velocity = 1.f;
  config.min_z_velocity = -0.5f;
  config.max_xy_velocity_norm = 3.f;
  config.max_acceleration_norm = 4.f;
  config.max_jerk_norm = 20.f;

  TrajectorySimulator sim(config, state);

  // WHEN: we get the list of trajectories with 0 steps
  Eigen::Vector3f goal_dir(0.f, 1.f, 0.f);
  float sim_time = 10;
  std::vector<simulation_state> steps = sim.generate_trajectory(goal_dir, sim_time);

  // THEN: we should get a list where it moves in the goal direction
  EXPECT_NO_FATAL_FAILURE(expect_goes_in_goal_direction(goal_dir, state, steps));

  // AND: it should respect the configured limits
  EXPECT_NO_FATAL_FAILURE(expect_respects_limits(config, state, steps));

  // AND: it should, at the end, have reached the goal velocity
  simulation_state last = steps[steps.size() - 1];
  EXPECT_LT((last.velocity.normalized() - goal_dir.normalized()).norm(), 1e-5);
  EXPECT_LT(last.acceleration.norm(), 1e-5);
  EXPECT_GT(last.time, sim_time + state.time);

  //   print_states(state, steps);
}
