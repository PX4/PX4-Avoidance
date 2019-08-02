#include "safe_landing_planner/waypoint_generator.hpp"

#include "avoidance/common.h"

#include <ros/console.h>

namespace avoidance {
const Eigen::Vector3f nan_setpoint = Eigen::Vector3f(NAN, NAN, NAN);

using avoidance::SLPState;
std::string toString(SLPState state) {
  std::string state_str = "unknown";
  switch (state) {
    case SLPState::GOTO:
      state_str = "GOTO";
      break;
    case SLPState::ALTITUDE_CHANGE:
      state_str = "ALTITUDE CHANGE";
      break;
    case SLPState::LOITER:
      state_str = "LOITER";
      break;
    case SLPState::LAND:
      state_str = "LAND";
      break;
    case SLPState::EVALUATE_GRID:
      state_str = "EVALUATE_GRID";
      break;
  }
  return state_str;
}

WaypointGenerator::WaypointGenerator()
    : usm::StateMachine<SLPState>(SLPState::GOTO),
      publishTrajectorySetpoints_([](const Eigen::Vector3f &, const Eigen::Vector3f &, float, float) {
        ROS_ERROR("publishTrajectorySetpoints_ not set in WaypointGenerator");
      }) {
  for (size_t i = 0; i < mask_.rows(); i++) {
    for (size_t j = 0; j < mask_.cols(); j++) {
      mask_(i, j) = sqrt((i - smoothing_land_cell_) * (i - smoothing_land_cell_) +
                         (j - smoothing_land_cell_) * (j - smoothing_land_cell_)) < (smoothing_land_cell_ + 0.5f);
    }
  }
}

void WaypointGenerator::calculateWaypoint() {
  updateSLPState();
  iterateOnce();

  if (getState() != prev_slp_state_) {
    std::string state_str = toString(getState());
    ROS_INFO("\033[1;36m [WGN] Update to %s state \033[0m", state_str.c_str());
  }
}

void WaypointGenerator::updateSLPState() {
  if (update_smoothing_size_ || mask_.rows() != (smoothing_land_cell_ * 2 + 1)) {
    mask_.resize((smoothing_land_cell_ * 2 + 1), (smoothing_land_cell_ * 2 + 1));

    for (size_t i = 0; i < mask_.rows(); i++) {
      for (size_t j = 0; j < mask_.cols(); j++) {
        mask_(i, j) = sqrt((i - smoothing_land_cell_) * (i - smoothing_land_cell_) +
                           (j - smoothing_land_cell_) * (j - smoothing_land_cell_)) < (smoothing_land_cell_ + 0.5f);
      }
    }
    update_smoothing_size_ = false;
  }

  if (grid_slp_.land_.rows() != can_land_hysteresis_matrix_.rows()) {
    can_land_hysteresis_matrix_.resize(grid_slp_.land_.rows(), grid_slp_.land_.cols());
    can_land_hysteresis_matrix_.fill(0.0f);
    can_land_hysteresis_result_.resize(grid_slp_.land_.rows(), grid_slp_.land_.cols());
    can_land_hysteresis_result_.fill(0);
  }

  if (!is_land_waypoint_) {
    decision_taken_ = false;
    can_land_ = true;
    can_land_hysteresis_matrix_.fill(0.0f);
    explorarion_is_active_ = false;
    n_explored_pattern_ = -1;
    factor_exploration_ = 1.f;
    ROS_INFO("[WGN] Not a land waypoint");
  }

  return;
}

SLPState WaypointGenerator::chooseNextState(SLPState currentState, usm::Transition transition) {
  prev_slp_state_ = currentState;
  USM_TABLE(
      currentState, SLPState::GOTO,
      USM_STATE(transition, SLPState::GOTO, USM_MAP(usm::Transition::NEXT1, SLPState::ALTITUDE_CHANGE);
                USM_MAP(usm::Transition::NEXT2, SLPState::LOITER); USM_MAP(usm::Transition::NEXT3, SLPState::LAND));
      USM_STATE(transition, SLPState::ALTITUDE_CHANGE, USM_MAP(usm::Transition::NEXT1, SLPState::LOITER)); USM_STATE(
          transition, SLPState::LOITER, USM_MAP(usm::Transition::NEXT1, SLPState::LAND);
          USM_MAP(usm::Transition::NEXT2, SLPState::GOTO); USM_MAP(usm::Transition::NEXT3, SLPState::EVALUATE_GRID));
      USM_STATE(transition, SLPState::EVALUATE_GRID, USM_MAP(usm::Transition::NEXT1, SLPState::GOTO);
                USM_MAP(usm::Transition::NEXT2, SLPState::LOITER));
      USM_STATE(transition, SLPState::LAND, ));
}

usm::Transition WaypointGenerator::runCurrentState() {
  if (trigger_reset_) {
    trigger_reset_ = false;
    return usm::Transition::ERROR;
  }

  switch (getState()) {
    case SLPState::GOTO:
      return runGoTo();

    case SLPState::ALTITUDE_CHANGE:
      return runAltitudeChange();

    case SLPState::LOITER:
      return runLoiter();

    case SLPState::LAND:
      return runLand();

    case SLPState::EVALUATE_GRID:
      return runEvaluateGrid();
  }
}

usm::Transition WaypointGenerator::runGoTo() {
  decision_taken_ = false;
  if (explorarion_is_active_) {
    landing_radius_ = 0.5f;
    yaw_setpoint_ = avoidance::nextYaw(position_, goal_);
  }

  publishTrajectorySetpoints_(goal_, velocity_setpoint_, yaw_setpoint_, yaw_speed_setpoint_);
  ROS_INFO("\033[1;32m [WGN] goTo %f %f %f - %f %f %f \033[0m\n", goal_.x(), goal_.y(), goal_.z(),
           velocity_setpoint_.x(), velocity_setpoint_.y(), velocity_setpoint_.z());
  altitude_landing_area_percentile_ = landingAreaHeightPercentile(80.f);
  can_land_hysteresis_matrix_.fill(0.0f);

  ROS_INFO("[WGN] Landing Radius: xy  %f, z %f ", (goal_.topRows<2>() - position_.topRows<2>()).norm(),
           fabsf(position_.z() - altitude_landing_area_percentile_));

  if (found_land_area_in_grid_) {
    landing_radius_ = 0.5f;
    if (withinLandingRadius()) {
      found_land_area_in_grid_ = false;
      return usm::Transition::NEXT3;
    } else {
      return usm::Transition::REPEAT;
    }
  }
  if (withinLandingRadius() && !inVerticalRange() && is_land_waypoint_) {
    return usm::Transition::NEXT1;
  }

  if (withinLandingRadius() && inVerticalRange() && is_land_waypoint_) {
    start_seq_landing_decision_ = grid_slp_seq_;
    return usm::Transition::NEXT2;
  }
}

usm::Transition WaypointGenerator::runAltitudeChange() {
  if (prev_slp_state_ != SLPState::ALTITUDE_CHANGE) {
    yaw_setpoint_ = yaw_;
  }
  goal_.z() = NAN;
  altitude_landing_area_percentile_ = landingAreaHeightPercentile(80.f);
  float direction = (fabsf(position_.z() - altitude_landing_area_percentile_) - loiter_height_) < 0.f ? 1.f : -1.f;
  velocity_setpoint_.z() = direction * LAND_SPEED;
  publishTrajectorySetpoints_(goal_, velocity_setpoint_, yaw_setpoint_, yaw_speed_setpoint_);
  ROS_INFO("\033[1;35m [WGN] altitudeChange %f %f %f - %f %f %f \033[0m", goal_.x(), goal_.y(), goal_.z(),
           velocity_setpoint_.x(), velocity_setpoint_.y(), velocity_setpoint_.z());

  if (explorarion_is_active_) {
    landing_radius_ = 0.5f;
  }

  ROS_INFO("[WGN] Landing Radius: xy  %f, z %f ", (goal_.topRows<2>() - position_.topRows<2>()).norm(),
           fabsf(position_.z() - altitude_landing_area_percentile_));

  if (withinLandingRadius() && inVerticalRange() && is_land_waypoint_) {
    start_seq_landing_decision_ = grid_slp_seq_;
    return usm::Transition::NEXT1;
  }
  return usm::Transition::REPEAT;
}

usm::Transition WaypointGenerator::runLoiter() {
  if (prev_slp_state_ != SLPState::LOITER) {
    loiter_position_ = position_;
    loiter_yaw_ = yaw_;
  }

  if (prev_slp_state_ != SLPState::EVALUATE_GRID) {
    offset_center_ = Eigen::Vector2i(grid_slp_.land_.rows() / 2, grid_slp_.land_.cols() / 2);
  }
  std::cout << "grid_slp_seq_ " << grid_slp_seq_ << " start_seq_landing_decision_ " << start_seq_landing_decision_
            << std::endl;
  if (abs(grid_slp_seq_ - start_seq_landing_decision_) <= 20) {
    for (int i = 0; i < grid_slp_.land_.rows(); i++) {
      for (int j = 0; j < grid_slp_.land_.cols(); j++) {
        float cell_land_value = static_cast<float>(grid_slp_.land_(i, j));
        float can_land_hysteresis_matrix_prev = can_land_hysteresis_matrix_(i, j);
        can_land_hysteresis_matrix_(i, j) = (beta_ * can_land_hysteresis_matrix_prev) + (1.f - beta_) * cell_land_value;
      }
    }
  } else {
    decision_taken_ = true;

    can_land_hysteresis_matrix_ =
        (can_land_hysteresis_matrix_.array() <= can_land_thr_).select(0, can_land_hysteresis_matrix_);
    can_land_hysteresis_matrix_ =
        (can_land_hysteresis_matrix_.array() > can_land_thr_).select(1, can_land_hysteresis_matrix_);
    can_land_hysteresis_result_ = can_land_hysteresis_matrix_.template cast<int>();
    std::cout << can_land_hysteresis_result_ << std::endl;

    Eigen::Vector2i left_upper_corner =
        Eigen::Vector2i(offset_center_.x() - smoothing_land_cell_, offset_center_.y() - smoothing_land_cell_);
    can_land_ = evaluatePatch(left_upper_corner);
  }

  publishTrajectorySetpoints_(loiter_position_, nan_setpoint, loiter_yaw_, NAN);
  ROS_INFO("\033[1;34m [WGN] Loiter %f %f %f - nan nan nan \033[0m\n", loiter_position_.x(), loiter_position_.y(),
           loiter_position_.z());
  std::cout << "decision_taken_ " << decision_taken_ << " can_land_ " << can_land_ << " start_grid_exploration_ "
            << start_grid_exploration_ << std::endl;
  if (decision_taken_ && can_land_) {
    return usm::Transition::NEXT1;
  } else if (decision_taken_ && !can_land_ && start_grid_exploration_) {
    return usm::Transition::NEXT3;
  } else if (decision_taken_ && !can_land_) {
    if (!explorarion_is_active_) {
      exploration_anchor_ = loiter_position_;
      explorarion_is_active_ = true;
    }
    float offset_exploration_setpoint =
        spiral_width_ * factor_exploration_ * 2.f * static_cast<float>(smoothing_land_cell_) * grid_slp_.getCellSize();
    n_explored_pattern_++;
    if (n_explored_pattern_ == exploration_pattern.size()) {
      n_explored_pattern_ = 0;
      factor_exploration_ += 1.f;
    }
    goal_ = Eigen::Vector3f(
        exploration_anchor_.x() + offset_exploration_setpoint * exploration_pattern[n_explored_pattern_].x(),
        exploration_anchor_.y() + offset_exploration_setpoint * exploration_pattern[n_explored_pattern_].y(),
        exploration_anchor_.z());
    velocity_setpoint_ = nan_setpoint;
    start_grid_exploration_ = true;
    return usm::Transition::NEXT2;
  }
  return usm::Transition::REPEAT;
}

usm::Transition WaypointGenerator::runLand() {
  if (prev_slp_state_ == SLPState::GOTO) {
    loiter_position_ = position_;
    loiter_yaw_ = yaw_;
  }
  loiter_position_.z() = NAN;
  Eigen::Vector3f vel_sp = nan_setpoint;
  vel_sp.z() = -LAND_SPEED;
  publishTrajectorySetpoints_(loiter_position_, vel_sp, loiter_yaw_, NAN);
  ROS_INFO("\033[1;36m [WGN] Land %f %f %f - nan nan nan \033[0m\n", loiter_position_.x(), loiter_position_.y(),
           loiter_position_.z());
  return usm::Transition::REPEAT;
}

usm::Transition WaypointGenerator::runEvaluateGrid() {
  ROS_INFO("\033[1;31m [WGN] runEvaluateGrid \033[0m\n");
  publishTrajectorySetpoints_(loiter_position_, nan_setpoint, loiter_yaw_, NAN);

  Eigen::Vector2i center = Eigen::Vector2i(grid_slp_.land_.rows() / 2, grid_slp_.land_.cols() / 2);
  Eigen::Vector2i offset = Eigen::Vector2i(grid_slp_.land_.rows() / 2, grid_slp_.land_.cols() / 2);

  int n_iterations = (1 + (grid_slp_.land_.rows() - (2 * smoothing_land_cell_ + 1)) / stride_) / 2;
  for (int i = 1; i < n_iterations; i++) {
    for (int j = 0; j < exploration_pattern.size(); j++) {
      offset.x() = center.x() + exploration_pattern[j].x() * i * stride_ - smoothing_land_cell_;
      offset.y() = center.y() + exploration_pattern[j].y() * i * stride_ - smoothing_land_cell_;
      can_land_ = evaluatePatch(offset);

      if (can_land_) {
        found_land_area_in_grid_ = true;
        Eigen::Vector2f min, max;

        grid_slp_.getGridLimits(min, max);
        goal_ = Eigen::Vector3f(min.x() + grid_slp_.getCellSize() * (offset.x() + smoothing_land_cell_),
                                min.y() + grid_slp_.getCellSize() * (offset.y() + smoothing_land_cell_), position_.z());
        printf("min %f %f max %f %f \n", min.x(), min.y(), max.x(), max.y());
        ROS_INFO("found_land_area_in_grid_ index %d %d - %f %f %f \n", offset.x(), offset.y(), goal_.x(), goal_.y(),
                 goal_.z());
        return usm::Transition::NEXT1;
      }
    }
  }
  start_grid_exploration_ = false;
  return usm::Transition::NEXT2;
}

bool WaypointGenerator::evaluatePatch(Eigen::Vector2i &left_upper_corner) {
  Eigen::MatrixXi kernel(can_land_hysteresis_matrix_.rows(), can_land_hysteresis_matrix_.cols());
  kernel.fill(0);
  kernel.block(left_upper_corner.x(), left_upper_corner.y(), mask_.rows(), mask_.cols()) = mask_;

  Eigen::MatrixXi result = can_land_hysteresis_result_.cwiseProduct(kernel);
  if (result.sum() == mask_.sum()) {
    std::cout << kernel << "\n\n";
    std::cout << can_land_hysteresis_result_ << "\n\n";

    std::cout << result << "\n\n";
  }
  return result.sum() == mask_.sum();
}

bool WaypointGenerator::withinLandingRadius() {
  return (goal_.topRows<2>() - position_.topRows<2>()).norm() < landing_radius_;
}

bool WaypointGenerator::inVerticalRange() {
  return std::abs(std::abs(position_.z() - altitude_landing_area_percentile_) - loiter_height_) < vertical_range_error_;
}

float WaypointGenerator::landingAreaHeightPercentile(float percentile) {
  std::vector<float> altitude_landing_area((smoothing_land_cell_ * 2 + 1) * (smoothing_land_cell_ * 2 + 1));

  int offset_center = grid_slp_.land_.rows() / 2;
  for (int i = offset_center - smoothing_land_cell_; i <= offset_center + smoothing_land_cell_; i++) {
    for (int j = offset_center - smoothing_land_cell_; j <= offset_center + smoothing_land_cell_; j++) {
      int index = (smoothing_land_cell_ * 2 + 1) * (i - offset_center + smoothing_land_cell_) +
                  (j - offset_center + smoothing_land_cell_);
      altitude_landing_area[index] = grid_slp_.mean_(i, j);
    }
  }

  std::sort(altitude_landing_area.begin(), altitude_landing_area.end());
  int index = static_cast<int>(std::round(percentile / 100.f * altitude_landing_area.size()));
  return altitude_landing_area[index];
}
}
