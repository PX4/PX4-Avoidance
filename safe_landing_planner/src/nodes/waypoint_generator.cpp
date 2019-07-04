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
  }
  return state_str;
}

WaypointGenerator::WaypointGenerator()
    : usm::StateMachine<SLPState>(SLPState::GOTO),
      publishTrajectorySetpoints_([](const Eigen::Vector3f&,
                                     const Eigen::Vector3f&, float, float) {
        ROS_ERROR("publishTrajectorySetpoints_ not set in WaypointGenerator");
      }) {}

void WaypointGenerator::calculateWaypoint() {
  updateSLPState();
  iterateOnce();

  if (getState() != prev_slp_state_) {
    std::string state_str = toString(getState());
    ROS_INFO("\033[1;36m [WGN] Update to %s state \033[0m", state_str.c_str());
  }
}

void WaypointGenerator::updateSLPState() {
  if (update_smoothing_size_ || can_land_hysteresis_.size() == 0) {
    can_land_hysteresis_.resize((smoothing_land_cell_ * 2 + 1) *
                                (smoothing_land_cell_ * 2 + 1));
    std::fill(can_land_hysteresis_.begin(), can_land_hysteresis_.end(), 0.f);
    update_smoothing_size_ = false;
  }

  if (!is_land_waypoint_) {
    decision_taken_ = false;
    can_land_ = true;
    can_land_hysteresis_.reserve((smoothing_land_cell_ * 2 + 1) *
                                 (smoothing_land_cell_ * 2 + 1));
    std::fill(can_land_hysteresis_.begin(), can_land_hysteresis_.end(), 0.f);
    explorarion_is_active_ = false;
    n_explored_pattern_ = -1;
    factor_exploration_ = 1.f;
    ROS_INFO("[WGN] Not a land waypoint");
  }

  return;
}

SLPState WaypointGenerator::chooseNextState(SLPState currentState,
                                            usm::Transition transition) {
  prev_slp_state_ = currentState;
  USM_TABLE(
      currentState, SLPState::GOTO,
      USM_STATE(transition, SLPState::GOTO,
                USM_MAP(usm::Transition::NEXT1, SLPState::ALTITUDE_CHANGE);
                USM_MAP(usm::Transition::NEXT2, SLPState::LOITER));
      USM_STATE(transition, SLPState::ALTITUDE_CHANGE,
                USM_MAP(usm::Transition::NEXT1, SLPState::LOITER));
      USM_STATE(transition, SLPState::LOITER,
                USM_MAP(usm::Transition::NEXT1, SLPState::LAND);
                USM_MAP(usm::Transition::NEXT2, SLPState::GOTO));
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
  }
}

usm::Transition WaypointGenerator::runGoTo() {
  decision_taken_ = false;
  if (explorarion_is_active_) {
    landing_radius_ = 0.5f;
    yaw_setpoint_ = avoidance::nextYaw(position_, goal_);
  }
  publishTrajectorySetpoints_(goal_, velocity_setpoint_, yaw_setpoint_,
                              yaw_speed_setpoint_);
  ROS_INFO("\033[1;32m [WGN] goTo %f %f %f - %f %f %f \033[0m\n", goal_.x(),
           goal_.y(), goal_.z(), velocity_setpoint_.x(), velocity_setpoint_.y(),
           velocity_setpoint_.z());
  altitude_landing_area_percentile_ = landingAreaHeightPercentile(80.f);
  std::fill(can_land_hysteresis_.begin(), can_land_hysteresis_.end(), 0.f);

  ROS_INFO("[WGN] Landing Radius: xy  %f, z %f ",
           (goal_.topRows<2>() - position_.topRows<2>()).norm(),
           fabsf(position_.z() - altitude_landing_area_percentile_));

  if (withinLandingRadius() && !inVerticalRange() && is_land_waypoint_) {
    return usm::Transition::NEXT1;
  }

  if (withinLandingRadius() && inVerticalRange() && is_land_waypoint_) {
    start_seq_landing_decision_ = grid_slp_seq_;
    return usm::Transition::NEXT2;
  }
  return usm::Transition::REPEAT;
}

usm::Transition WaypointGenerator::runAltitudeChange() {
  if (prev_slp_state_ != SLPState::ALTITUDE_CHANGE) {
    yaw_setpoint_ = yaw_;
  }
  goal_.z() = NAN;
  altitude_landing_area_percentile_ = landingAreaHeightPercentile(80.f);
  float direction = (fabsf(position_.z() - altitude_landing_area_percentile_) -
                     loiter_height_) < 0.f
                        ? 1.f
                        : -1.f;
  velocity_setpoint_.z() = direction * LAND_SPEED;
  publishTrajectorySetpoints_(goal_, velocity_setpoint_, yaw_setpoint_,
                              yaw_speed_setpoint_);
  ROS_INFO("\033[1;35m [WGN] altitudeChange %f %f %f - %f %f %f \033[0m",
           goal_.x(), goal_.y(), goal_.z(), velocity_setpoint_.x(),
           velocity_setpoint_.y(), velocity_setpoint_.z());

  if (explorarion_is_active_) {
    landing_radius_ = 0.5f;
  }

  ROS_INFO("[WGN] Landing Radius: xy  %f, z %f ",
           (goal_.topRows<2>() - position_.topRows<2>()).norm(),
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

  int offset_center = grid_slp_.land_.rows() / 2;
  for (int i = offset_center - smoothing_land_cell_;
       i <= offset_center + smoothing_land_cell_; i++) {
    for (int j = offset_center - smoothing_land_cell_;
         j <= offset_center + smoothing_land_cell_; j++) {
      int index = (smoothing_land_cell_ * 2 + 1) *
                      (i - offset_center + smoothing_land_cell_) +
                  (j - offset_center + smoothing_land_cell_);
      float cell_land_value = static_cast<float>(grid_slp_.land_(i, j));
      float can_land_hysteresis_prev = can_land_hysteresis_[index];
      can_land_hysteresis_[index] =
          (beta_ * can_land_hysteresis_prev) + (1.f - beta_) * cell_land_value;
      std::cout << can_land_hysteresis_[index] << " ";
    }
    std::cout << "\n";
  }
  std::cout << "\n";

  if (abs(grid_slp_seq_ - start_seq_landing_decision_) > 20) {
    decision_taken_ = true;
    int land_counter = 0;
    for (int i = 0; i < can_land_hysteresis_.size(); i++) {
      if (can_land_hysteresis_[i] > can_land_thr_) {
        land_counter++;
      }
      can_land_ = (can_land_ && (can_land_hysteresis_[i] > can_land_thr_));
      if (can_land_ == 0 && land_counter == can_land_hysteresis_.size()) {
        can_land_ = 1;
        ROS_INFO("[WGN] Decision changed from can't land to can land!");
      }
    }
  }

  publishTrajectorySetpoints_(loiter_position_, nan_setpoint, loiter_yaw_, NAN);
  ROS_INFO("\033[1;34m [WGN] Loiter %f %f %f - nan nan nan \033[0m\n",
           loiter_position_.x(), loiter_position_.y(), loiter_position_.z());

  if (decision_taken_ && can_land_) {
    return usm::Transition::NEXT1;
  } else if (decision_taken_ && !can_land_) {
    if (!explorarion_is_active_) {
      exploration_anchor_ = loiter_position_;
      explorarion_is_active_ = true;
    }
    float offset_exploration_setpoint =
        spiral_width_ * factor_exploration_ * 2.f *
        static_cast<float>(smoothing_land_cell_) * grid_slp_.getCellSize();
    n_explored_pattern_++;
    if (n_explored_pattern_ == exploration_pattern.size()) {
      n_explored_pattern_ = 0;
      factor_exploration_ += 1.f;
    }
    goal_ =
        Eigen::Vector3f(exploration_anchor_.x() +
                            offset_exploration_setpoint *
                                exploration_pattern[n_explored_pattern_].x(),
                        exploration_anchor_.y() +
                            offset_exploration_setpoint *
                                exploration_pattern[n_explored_pattern_].y(),
                        exploration_anchor_.z());
    velocity_setpoint_ = nan_setpoint;
    return usm::Transition::NEXT2;
  }
  return usm::Transition::REPEAT;
}

usm::Transition WaypointGenerator::runLand() {
  loiter_position_.z() = NAN;
  Eigen::Vector3f vel_sp = nan_setpoint;
  vel_sp.z() = -LAND_SPEED;
  publishTrajectorySetpoints_(loiter_position_, vel_sp, loiter_yaw_, NAN);
  ROS_INFO("\033[1;36m [WGN] Land %f %f %f - nan nan nan \033[0m\n",
           loiter_position_.x(), loiter_position_.y(), loiter_position_.z());
  return usm::Transition::REPEAT;
}

bool WaypointGenerator::withinLandingRadius() {
  return (goal_.topRows<2>() - position_.topRows<2>()).norm() < landing_radius_;
}

bool WaypointGenerator::inVerticalRange() {
  return std::abs(std::abs(position_.z() - altitude_landing_area_percentile_) -
                  loiter_height_) < vertical_range_error_;
}

float WaypointGenerator::landingAreaHeightPercentile(float percentile) {
  std::vector<float> altitude_landing_area(can_land_hysteresis_.size());

  int offset_center = grid_slp_.land_.rows() / 2;
  for (int i = offset_center - smoothing_land_cell_;
       i <= offset_center + smoothing_land_cell_; i++) {
    for (int j = offset_center - smoothing_land_cell_;
         j <= offset_center + smoothing_land_cell_; j++) {
      int index = (smoothing_land_cell_ * 2 + 1) *
                      (i - offset_center + smoothing_land_cell_) +
                  (j - offset_center + smoothing_land_cell_);
      altitude_landing_area[index] = grid_slp_.mean_(i, j);
    }
  }

  std::sort(altitude_landing_area.begin(), altitude_landing_area.end());
  int index = static_cast<int>(
      std::round(percentile / 100.f * altitude_landing_area.size()));
  return altitude_landing_area[index];
}
}
