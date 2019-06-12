#include "safe_landing_planner/waypoint_generator_node.hpp"
#include "avoidance/common.h"
#include "safe_landing_planner/safe_landing_planner.hpp"
#include "tf/transform_datatypes.h"

namespace avoidance {
const Eigen::Vector3f nan_setpoint = Eigen::Vector3f(NAN, NAN, NAN);

WaypointGeneratorNode::WaypointGeneratorNode(const ros::NodeHandle &nh)
    : nh_(nh), spin_dt_(0.1) {
  dynamic_reconfigure::Server<
      safe_landing_planner::WaypointGeneratorNodeConfig>::CallbackType f;
  f = boost::bind(&WaypointGeneratorNode::dynamicReconfigureCallback, this, _1,
                  _2);
  server_.setCallback(f);

  pose_sub_ = nh_.subscribe<const geometry_msgs::PoseStamped &>(
      "/mavros/local_position/pose", 1,
      &WaypointGeneratorNode::positionCallback, this);
  trajectory_sub_ =
      nh_.subscribe("/mavros/trajectory/desired", 1,
                    &WaypointGeneratorNode::trajectoryCallback, this);
  mission_sub_ = nh_.subscribe("/mavros/mission/waypoints", 1,
                               &WaypointGeneratorNode::missionCallback, this);
  state_sub_ = nh_.subscribe("/mavros/state", 1,
                             &WaypointGeneratorNode::stateCallback, this);
  grid_sub_ =
      nh_.subscribe("/grid_slp", 1, &WaypointGeneratorNode::gridCallback, this);

  trajectory_pub_ = nh_.advertise<mavros_msgs::Trajectory>(
      "/mavros/trajectory/generated", 10);
  land_hysteresis_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("/land_hysteresis", 1);
  marker_goal_pub_ =
      nh_.advertise<visualization_msgs::Marker>("/goal_position", 1);
}

void WaypointGeneratorNode::startNode() {
  ros::TimerOptions timer_options(
      ros::Duration(spin_dt_),
      boost::bind(&WaypointGeneratorNode::cmdLoopCallback, this, _1),
      &cmdloop_queue_);
  cmdloop_timer_ = nh_.createTimer(timer_options);
  cmdloop_spinner_.reset(new ros::AsyncSpinner(1, &cmdloop_queue_));
  cmdloop_spinner_->start();
}

void WaypointGeneratorNode::cmdLoopCallback(const ros::TimerEvent &event) {
  ros::Time start_query_position = ros::Time::now();
  while (!grid_received_ && ros::ok()) {
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
  }

  calculateWaypoint();
  landingAreaVisualization();
  goalVisualization();
  grid_received_ = false;

  return;
}

void WaypointGeneratorNode::dynamicReconfigureCallback(
    safe_landing_planner::WaypointGeneratorNodeConfig &config, uint32_t level) {
  update_smoothing_size_ = false;
  beta_ = static_cast<float>(config.beta);
  landing_radius_ = static_cast<float>(config.landing_radius);
  can_land_thr_ = static_cast<float>(config.can_land_thr);
  loiter_height_ = static_cast<float>(config.loiter_height);
  smoothing_land_cell_ = config.smoothing_land_cell;
  vertical_range_error_ = static_cast<float>(config.vertical_range_error);
  spiral_width_ = static_cast<float>(config.spiral_width);

  if (can_land_hysteresis_.size() !=
      ((smoothing_land_cell_ * 2) * (smoothing_land_cell_ * 2))) {
    update_smoothing_size_ = true;
  }
}

void WaypointGeneratorNode::positionCallback(
    const geometry_msgs::PoseStamped &msg) {
  position_ = avoidance::toEigen(msg.pose.position);
  double roll, pitch, yaw;

  tf::Quaternion q(msg.pose.orientation.x, msg.pose.orientation.y,
                   msg.pose.orientation.z, msg.pose.orientation.w);
  tf::Matrix3x3 mat(q);
  mat.getRPY(roll, pitch, yaw);
  yaw_ = static_cast<float>(yaw);
  ROS_INFO("[WGN] Current position %f %f %f", msg.pose.position.x,
           msg.pose.position.y, msg.pose.position.z);
}

void WaypointGeneratorNode::trajectoryCallback(
    const mavros_msgs::Trajectory &msg) {
  bool update =
      ((avoidance::toEigen(msg.point_2.position) - goal_visualization_).norm() >
       0.01) ||
      (!std::isfinite(goal_.x()) && !std::isfinite(goal_.y()));

  if (update && msg.point_valid[0] == true) {
    goal_ = avoidance::toEigen(msg.point_1.position);
    ROS_INFO(
        "\033[1;33m [WGN] Set New goal from FCU %f %f %f - nan nan nan \033[0m",
        goal_.x(), goal_.y(), goal_.z());
    velocity_setpoint_ = avoidance::toEigen(msg.point_1.velocity);
  }
  if (msg.point_valid[1] == true) {
    goal_visualization_ = avoidance::toEigen(msg.point_2.position);
    yaw_setpoint_ = msg.point_2.yaw;
    yaw_speed_setpoint_ = msg.point_2.yaw_rate;
  }
}

void WaypointGeneratorNode::missionCallback(
    const mavros_msgs::WaypointList &msg) {
  is_land_waypoint_ = false;
  for (auto waypoint : msg.waypoints) {
    if (waypoint.is_current && waypoint.command == 21) {
      is_land_waypoint_ = true;
    }
  }
}

void WaypointGeneratorNode::stateCallback(const mavros_msgs::State &msg) {
  if (msg.mode == "AUTO.TAKEOFF") {
    slp_state_ = SLPState::goTo;
    is_land_waypoint_ = false;
  } else if (msg.mode == "AUTO.LAND") {
    is_land_waypoint_ = true;
  } else if (msg.mode == "AUTO.MISSION") {
    // is_land_waypoint_ is set trought the mission item type
  } else {
    is_land_waypoint_ = false;
    slp_state_ = SLPState::goTo;
  }

  if (msg.armed == false) {
    is_land_waypoint_ = false;
  }
}

void WaypointGeneratorNode::gridCallback(
    const safe_landing_planner::SLPGridMsg &msg) {
  grid_slp_seq_ = msg.header.seq;
  if (grid_slp_.getGridSize() != msg.grid_size ||
      grid_slp_.getCellSize() != msg.cell_size) {
    grid_slp_.resize(msg.grid_size, msg.cell_size);
  }

  for (int i = 0; i < msg.mean.layout.dim[0].size; i++) {
    for (int j = 0; j < msg.mean.layout.dim[1].size; j++) {
      grid_slp_.mean_(i, j) = msg.mean.data[grid_slp_.mean_.cols() * i + j];
      grid_slp_.land_(i, j) = msg.land.data[grid_slp_.land_.cols() * i + j];
    }
  }

  pos_index_.x() = static_cast<int>(msg.curr_pos_index.x);
  pos_index_.y() = static_cast<int>(msg.curr_pos_index.y);

  grid_slp_.setFilterLimits(position_);
  grid_received_ = true;
}

void WaypointGeneratorNode::calculateWaypoint() {
  updateSLPState();

  switch (slp_state_) {
    case SLPState::goTo: {
      decision_taken_ = false;
      if (explorarion_is_active_) {
        landing_radius_ = 0.5f;
        yaw_setpoint_ = avoidance::nextYaw(position_, goal_);
      }
      publishTrajectorySetpoints(goal_, velocity_setpoint_, yaw_setpoint_,
                                 yaw_speed_setpoint_);
      ROS_INFO("\033[1;32m [WGN] goTo %f %f %f - %f %f %f \033[0m\n", goal_.x(),
               goal_.y(), goal_.z(), velocity_setpoint_.x(),
               velocity_setpoint_.y(), velocity_setpoint_.z());

      is_within_landing_radius_ =
          (goal_.topRows<2>() - position_.topRows<2>()).norm() <
          landing_radius_;
      in_land_vertical_range_ =
          fabsf(fabsf(position_.z() -
                      grid_slp_.mean_(pos_index_.x(), pos_index_.y())) -
                loiter_height_) < vertical_range_error_;
      ROS_INFO("[WGN] Landing Radius: xy  %f, z %f ",
               (goal_.topRows<2>() - position_.topRows<2>()).norm(),
               fabsf(position_.z() -
                     grid_slp_.mean_(pos_index_.x(), pos_index_.y())));
      std::cout << fabsf(
                       fabsf(position_.z() -
                             grid_slp_.mean_(pos_index_.x(), pos_index_.y())) -
                       loiter_height_)
                << std::endl;
      if (is_within_landing_radius_ && !in_land_vertical_range_ &&
          is_land_waypoint_ && !std::isfinite(velocity_setpoint_.z())) {
        prev_slp_state_ = SLPState::goTo;
        slp_state_ = SLPState::altitudeChange;
        ROS_INFO("\033[1;35m [WGN] Update to altitudeChange State \033[0m");
      }

      if (is_within_landing_radius_ && in_land_vertical_range_ &&
          is_land_waypoint_) {
        start_seq_landing_decision_ = grid_slp_seq_;
        prev_slp_state_ = SLPState::goTo;
        slp_state_ = SLPState::loiter;
        ROS_INFO("\033[1;34m [WGN] Update to Loiter State \033[0m");
      }
      break;
    }

    case SLPState::altitudeChange: {
      if (prev_slp_state_ != SLPState::altitudeChange) {
        yaw_setpoint_ = yaw_;
      }
      goal_.z() = NAN;
      float direction =
          (fabsf(position_.z() -
                 grid_slp_.mean_(pos_index_.x(), pos_index_.y())) -
           loiter_height_) < 0.f
              ? 1.f
              : -1.f;
      velocity_setpoint_.z() = direction * 0.5f;
      publishTrajectorySetpoints(goal_, velocity_setpoint_, yaw_setpoint_,
                                 yaw_speed_setpoint_);
      ROS_INFO("\033[1;35m [WGN] altitudeChange %f %f %f - %f %f %f \033[0m",
               goal_.x(), goal_.y(), goal_.z(), velocity_setpoint_.x(),
               velocity_setpoint_.y(), velocity_setpoint_.z());

      if (explorarion_is_active_) {
        landing_radius_ = 0.5f;
      }
      is_within_landing_radius_ =
          (goal_.topRows<2>() - position_.topRows<2>()).norm() <
          landing_radius_;
      in_land_vertical_range_ =
          fabsf(fabsf(position_.z() -
                      grid_slp_.mean_(pos_index_.x(), pos_index_.y())) -
                loiter_height_) < vertical_range_error_;
      ROS_INFO("[WGN] Landing Radius: xy  %f, z %f ",
               (goal_.topRows<2>() - position_.topRows<2>()).norm(),
               fabsf(position_.z() -
                     grid_slp_.mean_(pos_index_.x(), pos_index_.y())));

      if (is_within_landing_radius_ && in_land_vertical_range_ &&
          is_land_waypoint_) {
        start_seq_landing_decision_ = grid_slp_seq_;
        prev_slp_state_ = SLPState::altitudeChange;
        slp_state_ = SLPState::loiter;
        ROS_INFO("\033[1;34m [WGN] Update to Loiter State \033[0m");
      }
      break;
    }

    case SLPState::loiter: {
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
          can_land_hysteresis_[index] = (beta_ * can_land_hysteresis_prev) +
                                        (1.f - beta_) * cell_land_value;
        }
      }

      if (abs(grid_slp_seq_ - start_seq_landing_decision_) > 20) {
        decision_taken_ = true;
        int land_counter = 0;
        for (int i = 0; i < can_land_hysteresis_.size(); i++) {
          std::cout << can_land_hysteresis_[i] << " ";
          if (can_land_hysteresis_[i] > can_land_thr_) {
            land_counter++;
          }
          can_land_ = (can_land_ && (can_land_hysteresis_[i] > can_land_thr_));
          if (can_land_ == 0 && land_counter == can_land_hysteresis_.size()) {
            can_land_ = 1;
            decision_taken_ = false;
            in_land_vertical_range_ = false;
            ROS_INFO("[WGN] Decision changed from can't land to can land!");
          }
        }
      }

      publishTrajectorySetpoints(loiter_position_, nan_setpoint, loiter_yaw_,
                                 NAN);
      ROS_INFO("\033[1;34m [WGN] Loiter %f %f %f - nan nan nan \033[0m\n",
               loiter_position_.x(), loiter_position_.y(),
               loiter_position_.z());

      if (decision_taken_ && can_land_) {
        ROS_INFO("\033[1;36m [WGN] Update to Land State \033[0m");
        slp_state_ = SLPState::land;
      }

      if (decision_taken_ && !can_land_) {
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
        goal_ = Eigen::Vector3f(
            exploration_anchor_.x() +
                offset_exploration_setpoint *
                    exploration_pattern[n_explored_pattern_].x(),
            exploration_anchor_.y() +
                offset_exploration_setpoint *
                    exploration_pattern[n_explored_pattern_].y(),
            exploration_anchor_.z());
        velocity_setpoint_ = nan_setpoint;
        slp_state_ = SLPState::goTo;
        ROS_INFO("\033[1;32m [WGN] Update to goTo State \033[0m");
      }

      break;
    }

    case SLPState::land:
      loiter_position_.z() = NAN;
      vel_sp = nan_setpoint;
      vel_sp.z() = -0.5f;
      publishTrajectorySetpoints(loiter_position_, vel_sp, loiter_yaw_, NAN);
      ROS_INFO("\033[1;36m [WGN] Land %f %f %f - nan nan nan \033[0m\n",
               loiter_position_.x(), loiter_position_.y(),
               loiter_position_.z());
      slp_state_ = SLPState::land;
      break;
  }
}

void WaypointGeneratorNode::updateSLPState() {
  if (update_smoothing_size_) {
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
    slp_state_ = SLPState::goTo;
    explorarion_is_active_ = false;
    ROS_INFO("[WGN] Not a land waypoint");
  }

  return;
}

void WaypointGeneratorNode::publishTrajectorySetpoints(
    const Eigen::Vector3f &pos_sp, const Eigen::Vector3f &vel_sp, float yaw_sp,
    float yaw_speed_sp) {
  mavros_msgs::Trajectory setpoint;
  setpoint.header.stamp = ros::Time::now();
  setpoint.header.frame_id = "local_origin";
  setpoint.type = 0;  // MAV_TRAJECTORY_REPRESENTATION::WAYPOINTS
  setpoint.point_1.position.x = pos_sp.x();
  setpoint.point_1.position.y = pos_sp.y();
  setpoint.point_1.position.z = pos_sp.z();
  setpoint.point_1.velocity.x = vel_sp.x();
  setpoint.point_1.velocity.y = vel_sp.y();
  setpoint.point_1.velocity.z = vel_sp.z();
  setpoint.point_1.acceleration_or_force.x = NAN;
  setpoint.point_1.acceleration_or_force.y = NAN;
  setpoint.point_1.acceleration_or_force.z = NAN;
  setpoint.point_1.yaw = yaw_sp;
  setpoint.point_1.yaw_rate = yaw_speed_sp;

  fillUnusedTrajectorySetpoints(setpoint.point_2);
  fillUnusedTrajectorySetpoints(setpoint.point_3);
  fillUnusedTrajectorySetpoints(setpoint.point_4);
  fillUnusedTrajectorySetpoints(setpoint.point_5);

  setpoint.time_horizon = {NAN, NAN, NAN, NAN, NAN};

  bool xy_pos_sp_valid = std::isfinite(setpoint.point_1.position.x) &&
                         std::isfinite(setpoint.point_1.position.y);
  bool xy_vel_sp_valid = std::isfinite(setpoint.point_1.velocity.x) &&
                         std::isfinite(setpoint.point_1.velocity.y);

  if ((xy_pos_sp_valid || xy_vel_sp_valid) &&
      (std::isfinite(setpoint.point_1.position.z ||
                     std::isfinite(setpoint.point_1.velocity.z)))) {
    setpoint.point_valid = {true, false, false, false, false};
  } else {
    setpoint.point_valid = {false, false, false, false, false};
  }

  trajectory_pub_.publish(setpoint);
}

void WaypointGeneratorNode::fillUnusedTrajectorySetpoints(
    mavros_msgs::PositionTarget &point) {
  point.position.x = NAN;
  point.position.y = NAN;
  point.position.z = NAN;
  point.velocity.x = NAN;
  point.velocity.y = NAN;
  point.velocity.z = NAN;
  point.acceleration_or_force.x = NAN;
  point.acceleration_or_force.y = NAN;
  point.acceleration_or_force.z = NAN;
  point.yaw = NAN;
  point.yaw_rate = NAN;
}

void WaypointGeneratorNode::landingAreaVisualization() {
  visualization_msgs::MarkerArray marker_array;

  float cell_size = grid_slp_.getCellSize();
  visualization_msgs::Marker cell;
  cell.header.frame_id = "local_origin";
  cell.header.stamp = ros::Time::now();
  cell.id = 0;
  cell.type = visualization_msgs::Marker::CUBE;
  cell.action = visualization_msgs::Marker::ADD;
  cell.pose.orientation.x = 0.0;
  cell.pose.orientation.y = 0.0;
  cell.pose.orientation.z = 0.0;
  cell.pose.orientation.w = 1.0;
  cell.scale.x = cell_size;
  cell.scale.y = cell_size;
  cell.scale.z = 1.0;
  cell.color.a = 0.5;
  cell.scale.z = 0.1;
  cell.color.b = 0.0;

  Eigen::Vector2f grid_min, grid_max;
  grid_slp_.getGridLimits(grid_min, grid_max);
  int offset = grid_slp_.land_.rows() / 2;
  float hysteresis_max_value = 1.0f;
  float hysteresis_min_value = 0.0f;
  float range_max = 360.f;
  float range_min = 0.f;
  int counter = 0;

  for (size_t i = 0; i < grid_slp_.getRowColSize(); i++) {
    for (size_t j = 0; j < grid_slp_.getRowColSize(); j++) {
      if (i >= (offset - smoothing_land_cell_) &&
          i < (offset + smoothing_land_cell_) &&
          j >= (offset - smoothing_land_cell_) &&
          j < (offset + smoothing_land_cell_)) {
        cell.pose.position.x =
            (i * cell_size) + grid_min.x() + (cell_size / 2.f);
        cell.pose.position.y =
            (j * cell_size) + grid_min.y() + (cell_size / 2.f);
        cell.pose.position.z = 1.0;
        if (can_land_hysteresis_[counter] > can_land_thr_) {
          cell.color.r = 0.0;
          cell.color.g = 1.0;
        } else {
          cell.color.r = 1.0;
          cell.color.g = 0.0;
        }
        cell.color.a = 0.5;
        counter++;

        marker_array.markers.push_back(cell);
        cell.id += 1;
      }
    }
  }
  land_hysteresis_pub_.publish(marker_array);
}

void WaypointGeneratorNode::goalVisualization() {
  visualization_msgs::Marker m;
  static int id = 0;
  m.header.frame_id = "local_origin";
  m.header.stamp = ros::Time::now();
  m.type = visualization_msgs::Marker::SPHERE;
  m.action = visualization_msgs::Marker::ADD;
  m.scale.x = 0.5;
  m.scale.y = 0.5;
  m.scale.z = 0.5;
  m.color.a = 1.0;
  m.color.r = 1.0;
  m.color.g = 1.0;
  m.color.b = 0.0;

  m.lifetime = ros::Duration();
  m.id = id;
  id++;
  m.pose.position.x = goal_.x();
  m.pose.position.y = goal_.y();
  m.pose.position.z = goal_.z();

  marker_goal_pub_.publish(m);
}
}

int main(int argc, char **argv) {
  using namespace avoidance;
  ros::init(argc, argv, "waypoint_generator_node");
  ros::NodeHandle nh("~");

  WaypointGeneratorNode NodeWG(nh);
  NodeWG.startNode();
  while (ros::ok()) {
    ros::Duration(1.0).sleep();
  }

  return 0;
}
