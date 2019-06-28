#include "safe_landing_planner/waypoint_generator.hpp"

#include "avoidance/common.h"

#include <ros/console.h>

namespace avoidance {
const Eigen::Vector3f nan_setpoint = Eigen::Vector3f(NAN, NAN, NAN);

using avoidance::SLPState;
std::string toString(SLPState state) {
  std::string state_str = "unknown";
  switch (state) {
    case SLPState::goTo:
      state_str = "GOTO";
      break;
    case SLPState::altitudeChange:
      state_str = "ALTITUDE CHANGE";
      break;
    case SLPState::loiter:
      state_str = "LOITER";
      break;
    case SLPState::land:
      state_str = "LAND";
      break;
  }
  return state_str;
}

WaypointGenerator::WaypointGenerator()
    : usm::StateMachine<SLPState>(SLPState::goTo),
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
    can_land_hysteresis_.resize((smoothing_land_cell_ * 2) *
                                (smoothing_land_cell_ * 2));
    std::fill(can_land_hysteresis_.begin(), can_land_hysteresis_.end(), 0.f);
    update_smoothing_size_ = false;
  }

  if (!is_land_waypoint_) {
    decision_taken_ = false;
    can_land_ = true;
    can_land_hysteresis_.reserve((smoothing_land_cell_ * 2) *
                                 (smoothing_land_cell_ * 2));
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
  USM_TABLE(currentState, SLPState::goTo,
            USM_STATE(transition, SLPState::goTo,
                      USM_MAP(usm::NEXT1, SLPState::altitudeChange);
                      USM_MAP(usm::NEXT2, SLPState::loiter));
            USM_STATE(transition, SLPState::altitudeChange,
                      USM_MAP(usm::NEXT1, SLPState::loiter));
            USM_STATE(transition, SLPState::loiter,
                      USM_MAP(usm::NEXT1, SLPState::land);
                      USM_MAP(usm::NEXT2, SLPState::goTo));
            USM_STATE(transition, SLPState::land, ));
}

usm::Transition WaypointGenerator::runCurrentState(SLPState currentState) {
  if (trigger_reset_) {
    trigger_reset_ = false;
    return usm::ERROR;
  }

  switch (currentState) {
    case SLPState::goTo:
      return runGoTo();

    case SLPState::altitudeChange:
      return runAltitudeChange();

    case SLPState::loiter:
      return runLoiter();

    case SLPState::land:
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

  ROS_INFO(
      "[WGN] Landing Radius: xy  %f, z %f ",
      (goal_.topRows<2>() - position_.topRows<2>()).norm(),
      fabsf(position_.z() - grid_slp_.mean_(pos_index_.x(), pos_index_.y())));

  if (withinLandingRadius() && !inVerticalRange() && is_land_waypoint_ &&
      !std::isfinite(velocity_setpoint_.z())) {
    return usm::NEXT1;
  }

  if (withinLandingRadius() && inVerticalRange() && is_land_waypoint_) {
    start_seq_landing_decision_ = grid_slp_seq_;
    return usm::NEXT2;
  }
  return usm::REPEAT;
}

usm::Transition WaypointGenerator::runAltitudeChange() {
  if (prev_slp_state_ != SLPState::altitudeChange) {
    yaw_setpoint_ = yaw_;
  }
  goal_.z() = NAN;
  float direction =
      (fabsf(position_.z() - grid_slp_.mean_(pos_index_.x(), pos_index_.y())) -
       loiter_height_) < 0.f
          ? 1.f
          : -1.f;
  velocity_setpoint_.z() = direction * 0.5f;
  publishTrajectorySetpoints_(goal_, velocity_setpoint_, yaw_setpoint_,
                              yaw_speed_setpoint_);
  ROS_INFO("\033[1;35m [WGN] altitudeChange %f %f %f - %f %f %f \033[0m",
           goal_.x(), goal_.y(), goal_.z(), velocity_setpoint_.x(),
           velocity_setpoint_.y(), velocity_setpoint_.z());

  if (explorarion_is_active_) {
    landing_radius_ = 0.5f;
  }

  ROS_INFO(
      "[WGN] Landing Radius: xy  %f, z %f ",
      (goal_.topRows<2>() - position_.topRows<2>()).norm(),
      fabsf(position_.z() - grid_slp_.mean_(pos_index_.x(), pos_index_.y())));

  if (withinLandingRadius() && inVerticalRange() && is_land_waypoint_) {
    start_seq_landing_decision_ = grid_slp_seq_;
    return usm::NEXT1;
  }
  return usm::REPEAT;
}

usm::Transition WaypointGenerator::runLoiter() {
  if (prev_slp_state_ != SLPState::loiter) {
    loiter_position_ = position_;
    loiter_yaw_ = yaw_;
  }

  int offset_center = grid_slp_.land_.rows() / 2;
  for (int i = offset_center - smoothing_land_cell_;
       i < offset_center + smoothing_land_cell_; i++) {
    for (int j = offset_center - smoothing_land_cell_;
         j < offset_center + smoothing_land_cell_; j++) {
      int index = (smoothing_land_cell_ * 2) *
                      (i - offset_center + smoothing_land_cell_) +
                  (j - offset_center + smoothing_land_cell_);
      float cell_land_value = static_cast<float>(grid_slp_.land_(i, j));
      float can_land_hysteresis_prev = can_land_hysteresis_[index];
      can_land_hysteresis_[index] =
          (beta_ * can_land_hysteresis_prev) + (1.f - beta_) * cell_land_value;
    }
  }

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
    return usm::NEXT1;
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
    return usm::NEXT2;
  }
  return usm::REPEAT;
}

usm::Transition WaypointGenerator::runLand() {
  loiter_position_.z() = NAN;
  vel_sp = nan_setpoint;
  vel_sp.z() = -0.5f;
  publishTrajectorySetpoints_(loiter_position_, vel_sp, loiter_yaw_, NAN);
  ROS_INFO("\033[1;36m [WGN] Land %f %f %f - nan nan nan \033[0m\n",
           loiter_position_.x(), loiter_position_.y(), loiter_position_.z());
  return usm::REPEAT;
}

bool WaypointGenerator::withinLandingRadius() {
  return (goal_.topRows<2>() - position_.topRows<2>()).norm() < landing_radius_;
}

bool WaypointGenerator::inVerticalRange() {
  return std::abs(std::abs(position_.z() -
                           grid_slp_.mean_(pos_index_.x(), pos_index_.y())) -
                  loiter_height_) < vertical_range_error_;
}
}
