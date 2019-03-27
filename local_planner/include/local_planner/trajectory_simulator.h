#pragma once

#include <eigen3/Eigen/Core>

namespace avoidance {

struct simulation_state {
  float time = NAN;
  Eigen::Vector3f position = NAN * Eigen::Vector3f::Ones();
  Eigen::Vector3f velocity = NAN * Eigen::Vector3f::Ones();
  Eigen::Vector3f acceleration = NAN * Eigen::Vector3f::Ones();
};

struct simulation_limits {
  float max_z_velocity = NAN;
  float min_z_velocity = NAN;
  float max_xy_velocity_norm = NAN;
  float max_acceleration_norm = NAN;
  float max_jerk_norm = NAN;
};

std::vector<simulation_state> run_steps(const simulation_limits& config,
                                        const simulation_state& start,
                                        const Eigen::Vector3f& goal_direction,
                                        float step_time, int num_steps);

template <int N>
Eigen::Matrix<float, N, 1> norm_clamp(const Eigen::Matrix<float, N, 1>& val,
                                      float max_norm) {
  float norm_sq = val.squaredNorm();
  if (norm_sq > max_norm * max_norm)
    return val * (max_norm / std::sqrt(norm_sq));
  else
    return val;
}
}
