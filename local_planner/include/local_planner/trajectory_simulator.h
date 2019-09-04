#pragma once

#include <eigen3/Eigen/Core>
#include <vector>

namespace avoidance {

struct simulation_state {
  simulation_state(float t = NAN, const Eigen::Vector3f& p = NAN * Eigen::Vector3f::Zero(),
                   const Eigen::Vector3f& v = NAN * Eigen::Vector3f::Zero(),
                   const Eigen::Vector3f& a = NAN * Eigen::Vector3f::Zero())
      : time(t), position(p), velocity(v), acceleration(a){};
  float time;
  Eigen::Vector3f position;
  Eigen::Vector3f velocity;
  Eigen::Vector3f acceleration;
};

struct simulation_limits {
  simulation_limits(float max_z, float min_z, float max_xy_n, float max_acc_n, float max_jerk_n)
      : max_z_velocity(max_z),
        min_z_velocity(min_z),
        max_xy_velocity_norm(max_xy_n),
        max_acceleration_norm(max_acc_n),
        max_jerk_norm(max_jerk_n){};
  simulation_limits(){};
  float max_z_velocity = NAN;
  float min_z_velocity = NAN;
  float max_xy_velocity_norm = NAN;
  float max_acceleration_norm = NAN;
  float max_jerk_norm = NAN;
};

class TrajectorySimulator {
 public:
  TrajectorySimulator(const simulation_limits& config, const simulation_state& start, float step_time = 0.1f);

  std::vector<simulation_state> generate_trajectory(const Eigen::Vector3f& goal_direction, float simulation_duration);
  simulation_state generate_trajectory_endpoint(const Eigen::Vector3f& goal_direction, float simulation_duration);

 protected:
  const simulation_limits config_;
  const simulation_state start_;
  const float step_time_;

  static simulation_state simulate_step_constant_jerk(const simulation_state& state, const Eigen::Vector3f& jerk,
                                                      float step_time);

  static Eigen::Vector3f jerk_for_velocity_setpoint(float P_constant, float D_constant, float max_jerk_norm,
                                                    const Eigen::Vector3f& desired_velocity,
                                                    const simulation_state& state);

 private:
  simulation_state generate_trajectory(const Eigen::Vector3f& goal_direction, int num_steps,
                                       std::vector<simulation_state>* timepoints);
};

// templated helper function
template <int N>
Eigen::Matrix<float, N, 1> norm_clamp(const Eigen::Matrix<float, N, 1>& val, float max_norm) {
  float norm_sq = val.squaredNorm();
  if (norm_sq > max_norm * max_norm)
    return val * (max_norm / std::sqrt(norm_sq));
  else
    return val;
}
}
