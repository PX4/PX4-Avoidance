#pragma once

#include <avoidance/usm.h>
#include <safe_landing_planner/grid.hpp>

#include <Eigen/Dense>

#include <functional>
#include <vector>

#include "avoidance/common.h"

namespace avoidance {

const std::vector<Eigen::Vector2f> exploration_pattern = {
    Eigen::Vector2f(1.f, 0.f),  Eigen::Vector2f(1.f, 1.f),   Eigen::Vector2f(0.f, 1.f),  Eigen::Vector2f(-1.f, 1.f),
    Eigen::Vector2f(-1.f, 0.f), Eigen::Vector2f(-1.f, -1.f), Eigen::Vector2f(0.f, -1.f), Eigen::Vector2f(1.f, -1.f)};

enum class SLPState { GOTO, LOITER, LAND, ALTITUDE_CHANGE, EVALUATE_GRID, GOTO_LAND };
std::string toString(SLPState state);  // for logging

static const float LAND_SPEED = 0.7f;

class WaypointGenerator : public usm::StateMachine<SLPState> {
 public:
  WaypointGenerator();
  virtual ~WaypointGenerator() = default;

  /**
  * @brief     computes the setpoints to be sent to the FCU
  **/
  void calculateWaypoint();

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
  float altitude_landing_area_percentile_ = -1.f;
  int smoothing_land_cell_ = 6;

  // state
  bool trigger_reset_ = false;
  SLPState prev_slp_state_ = SLPState::GOTO;

  bool is_land_waypoint_ = false;
  bool decision_taken_ = false;
  bool can_land_ = true;
  bool update_smoothing_size_ = false;
  bool explorarion_is_active_ = false;
  bool state_changed_ = false;
  int start_seq_landing_decision_ = 0;
  int grid_slp_seq_ = 0;
  int n_explored_pattern_ = -1;
  int stride_ = 1;

  Eigen::Vector3f position_ = Eigen::Vector3f(NAN, NAN, NAN);
  Eigen::Vector3f goal_ = Eigen::Vector3f(NAN, NAN, NAN);
  Eigen::Vector3f velocity_setpoint_ = Eigen::Vector3f(NAN, NAN, NAN);
  Eigen::Vector3f loiter_position_ = Eigen::Vector3f(NAN, NAN, NAN);
  Eigen::Vector3f exploration_anchor_ = Eigen::Vector3f(NAN, NAN, NAN);
  Eigen::Vector2i pos_index_ = Eigen::Vector2i::Zero();

  Eigen::MatrixXf mean_ = Eigen::MatrixXf(40, 40);
  Eigen::MatrixXi land_ = Eigen::MatrixXi(40, 40);
  Eigen::MatrixXf can_land_hysteresis_matrix_ = Eigen::MatrixXf::Zero(40, 40);
  Eigen::MatrixXi can_land_hysteresis_result_ = Eigen::MatrixXi::Zero(40, 40);
  Eigen::MatrixXi mask_ = Eigen::MatrixXi(13, 13);

  Grid grid_slp_ = Grid(10.f, 1.f);

  // outside world link
  std::function<void(const Eigen::Vector3f& pos_sp, const Eigen::Vector3f& vel_sp, float yaw_sp, float yaw_speed_sp)>
      publishTrajectorySetpoints_;

  /**
  * @brief     update the waypoint generator state based on the vehicle status
  **/
  void updateSLPState();

  /**
  * @brief iterate the statemachine
  */
  usm::Transition runCurrentState() override final;

  /**
  * @brief the setup of the statemachine
  */
  SLPState chooseNextState(SLPState currentState, usm::Transition transition) override final;

  usm::Transition runGoTo();
  usm::Transition runLoiter();
  usm::Transition runLand();
  usm::Transition runAltitudeChange();
  usm::Transition runEvaluateGrid();
  usm::Transition runGoToLand();
  /**
  * @brief     checks if the vehicle is within the xy acceptance radius of a
  *            waypoint
  * @returns   true, if the vehicle is within the xy acceptance radius of the
  *            landing waypoint
  **/
  bool withinLandingRadius();
  /**
  * @brief     checks if the vehicle is within the z acceptance to the loiter
  *            height from the ground under it
  * @returns   true, if the vehicle is within the z acceptance to the loiter
  *            height
  **/
  bool inVerticalRange();
  /**
  * @brief      compute the percentile height of the landing area
  * @params[in] percentile, percentile be computed
  * @returns    height of the landing area
  **/
  float landingAreaHeightPercentile(float percentile);

  bool evaluatePatch(Eigen::Vector2i& left_upper_corner);

  void initializeMask();
  friend class WaypointGeneratorNode;  // TODO make an API and get rid of this
};
}
