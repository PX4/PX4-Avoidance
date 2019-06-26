#pragma once

#include <avoidance/usm.h>
#include <safe_landing_planner/grid.hpp>

#include <Eigen/Dense>

#include <functional>

namespace avoidance {

const std::vector<Eigen::Vector2f> exploration_pattern = {
    Eigen::Vector2f(1.f, 0.f),  Eigen::Vector2f(1.f, 1.f),
    Eigen::Vector2f(0.f, 1.f),  Eigen::Vector2f(-1.f, 1.f),
    Eigen::Vector2f(-1.f, 0.f), Eigen::Vector2f(-1.f, -1.f),
    Eigen::Vector2f(0.f, -1.f), Eigen::Vector2f(1.f, -1.f)};

enum class SLPState {
  goTo,
  loiter,
  land,
  altitudeChange,
};
std::string toString(SLPState state);  // for logging

class WaypointGenerator final : public usm::StateMachine<SLPState> {
  WaypointGenerator();
  ~WaypointGenerator() = default;

 protected:
  // config
  float yaw_setpoint_ = NAN;
  float yaw_speed_setpoint_ = NAN;
  float loiter_yaw_ = NAN;
  float yaw_ = NAN;
  float beta_ = 0.9f;
  float landing_radius_ = 2.f;
  float can_land_thr_ = 0.4f;
  float loiter_height_ = 4.f;
  float factor_exploration_ = 1.f;
  float vertical_range_error_ = 1.f;
  float spiral_width_ = 2.f;
  int smoothing_land_cell_ = 2;

  // state
  bool trigger_reset_ = false;
  SLPState prev_slp_state_ = SLPState::goTo;

  bool is_land_waypoint_ = false;
  bool decision_taken_ = false;
  bool can_land_ = true;
  bool update_smoothing_size_ = false;
  bool explorarion_is_active_ = false;
  int start_seq_landing_decision_ = 0;
  int grid_slp_seq_ = 0;
  int n_explored_pattern_ = -1;

  Eigen::Vector3f position_ = Eigen::Vector3f(NAN, NAN, NAN);
  Eigen::Vector3f goal_ = Eigen::Vector3f(NAN, NAN, NAN);
  Eigen::Vector3f velocity_setpoint_ = Eigen::Vector3f(NAN, NAN, NAN);
  Eigen::Vector3f loiter_position_ = Eigen::Vector3f(NAN, NAN, NAN);
  Eigen::Vector3f exploration_anchor_ = Eigen::Vector3f(NAN, NAN, NAN);
  Eigen::Vector3f vel_sp = Eigen::Vector3f(NAN, NAN, NAN);
  Eigen::Vector2i pos_index_ = Eigen::Vector2i(NAN, NAN);

  Eigen::MatrixXf mean_ = Eigen::MatrixXf(40, 40);
  Eigen::MatrixXi land_ = Eigen::MatrixXi(40, 40);
  std::vector<float> can_land_hysteresis_;
  Grid grid_slp_ = Grid(10.f, 1.f);

  // outside world link
  std::function<void(const Eigen::Vector3f& pos_sp,
                     const Eigen::Vector3f& vel_sp, float yaw_sp,
                     float yaw_speed_sp)>
      publishTrajectorySetpoints_;

  /**
  * @brief     computes the setpoints to be sent to the FCU
  **/
  void calculateWaypoint();

  /**
  * @brief     decides if the desired setpoints received from the FCU should be
  *overwritten
  *            or sent back because no intervention is needed
  * @returns   true, overwtite setpoints
  **/
  void updateSLPState();

  /**
  * @brief iterate the statemachine
  */
  usm::Transition runCurrentState(SLPState currentState) override;

  /**
  * @brief the setup of the statemachine
  */
  SLPState chooseNextState(SLPState currentState,
                           usm::Transition transition) override;

  usm::Transition runGoTo();
  usm::Transition runLoiter();
  usm::Transition runLand();
  usm::Transition runAltitudeChange();

  bool withinLandingRadius();
  bool inVerticalRange();

  friend class WaypointGeneratorNode;  // TODO make an API and get rid of this
};
}
