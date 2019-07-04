#include "local_planner/trajectory_simulator.h"

#include <cfloat>

namespace {
Eigen::Vector3f xy_norm_z_clamp(const Eigen::Vector3f& val, float max_xy_norm, float min_z, float max_z) {
  Eigen::Vector3f result;
  result.topRows<2>() = avoidance::norm_clamp<2>(val.topRows<2>(), max_xy_norm);
  result.z() = std::min(max_z, std::max(min_z, val.z()));
  return result;
}

float sqr(float f) { return f * f; }
float cube(float f) { return f * f * f; }
}

namespace avoidance {

TrajectorySimulator::TrajectorySimulator(const avoidance::simulation_limits& config,
                                         const avoidance::simulation_state& start, float step_time)
    : config_(config), start_(start), step_time_(step_time) {}

std::vector<simulation_state> TrajectorySimulator::generate_trajectory(const Eigen::Vector3f& goal_direction,
                                                                       float simulation_duration) {
  int num_steps = static_cast<int>(std::ceil(simulation_duration / step_time_));
  std::vector<simulation_state> timepoints;
  timepoints.reserve(num_steps);

  const Eigen::Vector3f unit_goal = goal_direction.normalized();
  const Eigen::Vector3f desired_velocity =
      xy_norm_z_clamp(unit_goal * std::hypot(config_.max_xy_velocity_norm,
                                             unit_goal.z() > 0 ? config_.max_z_velocity : config_.min_z_velocity),
                      config_.max_xy_velocity_norm, config_.min_z_velocity, config_.max_z_velocity);

  // calculate P and D constants such that they hit the jerk limit when
  // doing accel from 0
  float max_accel_norm = std::min(2 * std::sqrt(config_.max_jerk_norm), config_.max_acceleration_norm);
  float P_constant =
      (std::sqrt(sqr(max_accel_norm) + config_.max_jerk_norm * desired_velocity.norm()) - max_accel_norm) /
      desired_velocity.norm() * 10;
  float D_constant = 2 * std::sqrt(P_constant);

  simulation_state run_state = start_;
  for (int i = 0; i < num_steps; i++) {
    float single_step_time = step_time_;
    const Eigen::Vector3f damped_jerk =
        jerk_for_velocity_setpoint(P_constant, D_constant, config_.max_jerk_norm, desired_velocity, run_state);

    // limit time step to not exceed the maximum acceleration, but clamp
    // jerk to 0 if at maximum acceleration already
    const Eigen::Vector3f requested_accel = run_state.acceleration + single_step_time * damped_jerk;
    Eigen::Vector3f jerk = damped_jerk;
    if (requested_accel.squaredNorm() > sqr(max_accel_norm)) {
      single_step_time =
          (max_accel_norm - run_state.acceleration.norm()) / damped_jerk.norm();  // Use a dot product here somewhere?
      if (single_step_time <= FLT_EPSILON || single_step_time > step_time_) {
        jerk = Eigen::Vector3f::Zero();
        single_step_time = step_time_;
      }
    }

    // update the state based on motion equations with the final jerk
    run_state = simulate_step_constant_jerk(run_state, jerk, single_step_time);
    timepoints.push_back(run_state);
  }

  return timepoints;
}

simulation_state TrajectorySimulator::simulate_step_constant_jerk(const simulation_state& state,
                                                                  const Eigen::Vector3f& jerk, float step_time) {
  simulation_state next_state;
  next_state.position = state.position + step_time * state.velocity + 0.5f * sqr(step_time) * state.acceleration +
                        (1.f / 6.f) * cube(step_time) * jerk;
  next_state.velocity = state.velocity + state.acceleration * step_time + 0.5f * sqr(step_time) * jerk;
  next_state.acceleration = state.acceleration + step_time * jerk;
  next_state.time = state.time + step_time;
  return next_state;
}

Eigen::Vector3f TrajectorySimulator::jerk_for_velocity_setpoint(float P_constant, float D_constant, float max_jerk_norm,
                                                                const Eigen::Vector3f& desired_velocity,
                                                                const simulation_state& state) {
  const Eigen::Vector3f desired_accel = Eigen::Vector3f::Zero();
  const Eigen::Vector3f accel_diff = desired_accel - state.acceleration;
  const Eigen::Vector3f velocity_diff = desired_velocity - state.velocity;

  const Eigen::Vector3f p = velocity_diff * P_constant;
  const Eigen::Vector3f d = accel_diff * D_constant;

  const Eigen::Vector3f damped_jerk = norm_clamp<3>(p + d, max_jerk_norm);
  return damped_jerk;
}
}
