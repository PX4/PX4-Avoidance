#include "safe_landing_planner/waypoint_generator_node.hpp"

#include "safe_landing_planner/safe_landing_planner.hpp"
#include "tf/transform_datatypes.h"

namespace avoidance {
const Eigen::Vector3f nan_setpoint = Eigen::Vector3f(NAN, NAN, NAN);

WaypointGeneratorNode::WaypointGeneratorNode(const ros::NodeHandle &nh) : nh_(nh), spin_dt_(0.1) {
  dynamic_reconfigure::Server<safe_landing_planner::WaypointGeneratorNodeConfig>::CallbackType f;
  f = boost::bind(&WaypointGeneratorNode::dynamicReconfigureCallback, this, _1, _2);
  server_.setCallback(f);

  pose_sub_ = nh_.subscribe<const geometry_msgs::PoseStamped &>("mavros/local_position/pose", 1,
                                                                &WaypointGeneratorNode::positionCallback, this);
  trajectory_sub_ = nh_.subscribe("mavros/trajectory/desired", 1, &WaypointGeneratorNode::trajectoryCallback, this);
  state_sub_ = nh_.subscribe("mavros/state", 1, &WaypointGeneratorNode::stateCallback, this);
  grid_sub_ = nh_.subscribe("grid_slp", 1, &WaypointGeneratorNode::gridCallback, this);

  trajectory_pub_ = nh_.advertise<mavros_msgs::Trajectory>("mavros/trajectory/generated", 10);
  land_hysteresis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("land_hysteresis", 1);
  marker_goal_pub_ = nh_.advertise<visualization_msgs::Marker>("goal_position", 1);

  waypointGenerator_.publishTrajectorySetpoints_ = [this](const Eigen::Vector3f &pos_sp, const Eigen::Vector3f &vel_sp,
                                                          float yaw_sp, float yaw_speed_sp) {
    publishTrajectorySetpoints(pos_sp, vel_sp, yaw_sp, yaw_speed_sp);
  };
}

void WaypointGeneratorNode::startNode() {
  ros::TimerOptions timer_options(ros::Duration(spin_dt_),
                                  boost::bind(&WaypointGeneratorNode::cmdLoopCallback, this, _1), &cmdloop_queue_);
  cmdloop_timer_ = nh_.createTimer(timer_options);
  cmdloop_spinner_.reset(new ros::AsyncSpinner(1, &cmdloop_queue_));
  cmdloop_spinner_->start();
}

void WaypointGeneratorNode::cmdLoopCallback(const ros::TimerEvent &event) {
  while (!grid_received_ && ros::ok()) {
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
  }

  waypointGenerator_.calculateWaypoint();
  landingAreaVisualization();
  goalVisualization();
  grid_received_ = false;

  return;
}

void WaypointGeneratorNode::dynamicReconfigureCallback(safe_landing_planner::WaypointGeneratorNodeConfig &config,
                                                       uint32_t level) {
  waypointGenerator_.beta_ = static_cast<float>(config.beta);
  waypointGenerator_.can_land_thr_ = static_cast<float>(config.can_land_thr);
  waypointGenerator_.loiter_height_ = static_cast<float>(config.loiter_height);
  waypointGenerator_.smoothing_land_cell_ = config.smoothing_land_cell;
  waypointGenerator_.vertical_range_error_ = static_cast<float>(config.vertical_range_error);
  waypointGenerator_.spiral_width_ = static_cast<float>(config.spiral_width);

  if (waypointGenerator_.mask_.rows() != ((waypointGenerator_.smoothing_land_cell_ * 2) + 1)) {
    waypointGenerator_.update_smoothing_size_ = true;
  }
}

void WaypointGeneratorNode::positionCallback(const geometry_msgs::PoseStamped &msg) {
  waypointGenerator_.position_ = avoidance::toEigen(msg.pose.position);
  double roll, pitch, yaw;

  tf::Quaternion q(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
  tf::Matrix3x3 mat(q);
  mat.getRPY(roll, pitch, yaw);
  waypointGenerator_.yaw_ = static_cast<float>(yaw);
  ROS_INFO("[WGN] Current position %f %f %f", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
}

void WaypointGeneratorNode::trajectoryCallback(const mavros_msgs::Trajectory &msg) {
  bool update = ((avoidance::toEigen(msg.point_2.position) - goal_visualization_).norm() > 0.01) ||
                waypointGenerator_.goal_.topRows<2>().array().hasNaN();

  if (update && msg.point_valid[0] == true) {
    waypointGenerator_.goal_ = avoidance::toEigen(msg.point_1.position);
    waypointGenerator_.velocity_setpoint_ = avoidance::toEigen(msg.point_1.velocity);
    waypointGenerator_.is_land_waypoint_ = (msg.command[1] == static_cast<int>(MavCommand::MAV_CMD_NAV_LAND));
    ROS_INFO_STREAM("\033[1;33m [WGN] Set New goal from FCU " << waypointGenerator_.goal_.transpose()
                                                              << " - nan nan nan \033[0m");
  }
  if (msg.point_valid[1] == true) {
    goal_visualization_ = avoidance::toEigen(msg.point_2.position);
    waypointGenerator_.yaw_setpoint_ = msg.point_2.yaw;
    waypointGenerator_.yaw_speed_setpoint_ = msg.point_2.yaw_rate;
  }
}

void WaypointGeneratorNode::stateCallback(const mavros_msgs::State &msg) {
  if (msg.mode == "AUTO.LAND") {
    waypointGenerator_.is_land_waypoint_ = true;
  } else if (msg.mode == "AUTO.MISSION") {
    // is_land_waypoint_ is set trought the mission item type
  } else {
    waypointGenerator_.is_land_waypoint_ = false;
    waypointGenerator_.trigger_reset_ = true;
  }

  if (msg.armed == false) {
    waypointGenerator_.is_land_waypoint_ = false;
    waypointGenerator_.trigger_reset_ = true;
  }
}

void WaypointGeneratorNode::gridCallback(const safe_landing_planner::SLPGridMsg &msg) {
  waypointGenerator_.grid_slp_seq_ = msg.header.seq;
  if (waypointGenerator_.grid_slp_.getGridSize() != msg.grid_size ||
      waypointGenerator_.grid_slp_.getCellSize() != msg.cell_size) {
    waypointGenerator_.grid_slp_.resize(msg.grid_size, msg.cell_size);
  }

  for (int i = 0; i < msg.mean.layout.dim[0].size; i++) {
    for (int j = 0; j < msg.mean.layout.dim[1].size; j++) {
      waypointGenerator_.grid_slp_.mean_(i, j) = msg.mean.data[msg.mean.layout.dim[1].size * i + j];
      waypointGenerator_.grid_slp_.land_(i, j) = msg.land.data[msg.mean.layout.dim[1].size * i + j];
    }
  }

  waypointGenerator_.pos_index_.x() = static_cast<int>(msg.curr_pos_index.x);
  waypointGenerator_.pos_index_.y() = static_cast<int>(msg.curr_pos_index.y);

  waypointGenerator_.grid_slp_.setFilterLimits(waypointGenerator_.position_);
  grid_received_ = true;
}

void WaypointGeneratorNode::publishTrajectorySetpoints(const Eigen::Vector3f &pos_sp, const Eigen::Vector3f &vel_sp,
                                                       float yaw_sp, float yaw_speed_sp) {
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

  bool xy_pos_sp_valid = std::isfinite(setpoint.point_1.position.x) && std::isfinite(setpoint.point_1.position.y);
  bool xy_vel_sp_valid = std::isfinite(setpoint.point_1.velocity.x) && std::isfinite(setpoint.point_1.velocity.y);

  if ((xy_pos_sp_valid || xy_vel_sp_valid) &&
      (std::isfinite(setpoint.point_1.position.z || std::isfinite(setpoint.point_1.velocity.z)))) {
    setpoint.point_valid = {true, false, false, false, false};
  } else {
    setpoint.point_valid = {false, false, false, false, false};
  }

  trajectory_pub_.publish(setpoint);
}

void WaypointGeneratorNode::fillUnusedTrajectorySetpoints(mavros_msgs::PositionTarget &point) {
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

  float cell_size = waypointGenerator_.grid_slp_.getCellSize();
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
  waypointGenerator_.grid_slp_.getGridLimits(grid_min, grid_max);
  int offset = waypointGenerator_.grid_slp_.land_.rows() / 2;

  Eigen::MatrixXi kernel(waypointGenerator_.can_land_hysteresis_matrix_.rows(),
                         waypointGenerator_.can_land_hysteresis_matrix_.cols());
  kernel.fill(0);
  kernel.block(20 - waypointGenerator_.smoothing_land_cell_, 20 - waypointGenerator_.smoothing_land_cell_,
               waypointGenerator_.mask_.rows(), waypointGenerator_.mask_.cols()) = waypointGenerator_.mask_;

  Eigen::MatrixXi result = waypointGenerator_.can_land_hysteresis_result_.cwiseProduct(kernel);

  int slc = waypointGenerator_.smoothing_land_cell_;

  for (size_t k = offset - slc; k <= offset + slc; k++) {
    for (size_t l = offset - slc; l <= offset + slc; l++) {
      cell.pose.position.x = grid_min.x() + (cell_size * k);
      cell.pose.position.y = grid_min.y() + (cell_size * l);
      cell.pose.position.z = 1;
      if (result(k, l)) {
        cell.color.r = 0.0;
        cell.color.g = 1.0;
      } else {
        cell.color.r = 1.0;
        cell.color.g = 0.0;
      }
      cell.color.a = 0.5;

      marker_array.markers.push_back(cell);
      cell.id += 1;
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
  m.pose.position = toPoint(waypointGenerator_.goal_);

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
