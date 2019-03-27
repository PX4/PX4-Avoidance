#include "local_planner/trajectory_simulator.h"

namespace avoidance {

std::vector<simulation_state> run_steps(const simulation_limits& config,
                                        const simulation_state& start,
                                        const Eigen::Vector3f& goal_direction,
                                        float step_time, int num_steps) {
  auto xy_norm_z_clamp = [](const Eigen::Vector3f& val, float max_xy_norm,
                            float min_z, float max_z) -> Eigen::Vector3f {
    Eigen::Vector3f result;
    result.topRows<2>() = norm_clamp<2>(val.topRows<2>(), max_xy_norm);
    result.z() = std::min(max_z, std::max(min_z, val.z()));
    return result;
  };

  simulation_state run_state = start;
  std::vector<simulation_state> timepoints;
  timepoints.reserve(num_steps);

  const Eigen::Vector3f unit_goal = goal_direction.normalized();
  const Eigen::Vector3f desired_velocity = xy_norm_z_clamp(
      unit_goal * std::hypot(config.max_xy_velocity_norm,
                             unit_goal.z() > 0 ? config.max_z_velocity
                                               : config.min_z_velocity),
      config.max_xy_velocity_norm, config.min_z_velocity,
      config.max_z_velocity);

  auto sqr = [](float f) -> float { return f * f; };
  auto cube = [](float f) -> float { return f * f * f; };
  // calculate P and D constants such that they hit the jerk limit when
  // doing accel from 0
  float P_constant = config.max_jerk_norm;
  float D_constant = 2 * std::sqrt(P_constant);

  for (int i = 0; i < num_steps; i++) {
    // determine what our jerk should be to control the velocity
    const Eigen::Vector3f velocity_diff = desired_velocity - run_state.velocity;
    const Eigen::Vector3f desired_accel = Eigen::Vector3f::Zero();
    const Eigen::Vector3f accel_diff = desired_accel - run_state.acceleration;

    const Eigen::Vector3f p = velocity_diff * P_constant;
    const Eigen::Vector3f d = accel_diff * D_constant;

    const Eigen::Vector3f damped_jerk =
        norm_clamp<3>(p + d, config.max_jerk_norm);

    // clamp the jerk to keep the accel within requested limits
    const Eigen::Vector3f requested_accel = run_state.acceleration +=
        step_time * damped_jerk;
    const Eigen::Vector3f clamped_accel =
        norm_clamp<3>(requested_accel, config.max_acceleration_norm);
    const Eigen::Vector3f jerk =
        (clamped_accel - run_state.acceleration) / step_time;

    // update the state based on motion equations with the final jerk
    run_state.position += step_time * run_state.velocity +
                          0.5f * sqr(step_time) * run_state.acceleration +
                          (1.f / 6.f) * cube(step_time) * jerk;
    run_state.velocity +=
        run_state.acceleration * step_time + 0.5f * sqr(step_time) * jerk;
    run_state.acceleration += step_time * jerk;
    run_state.time += step_time;

    timepoints.push_back(run_state);
  }
  return timepoints;
}
}
