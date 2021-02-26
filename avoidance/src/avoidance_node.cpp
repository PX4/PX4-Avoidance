#include "avoidance/avoidance_node.h"

#include <thread>

using namespace std::chrono_literals;

namespace avoidance {

AvoidanceNode::AvoidanceNode()
    : cmdloop_dt_(100ms),
      statusloop_dt_(200ms),
      timeout_termination_(15000000000000ns), // ns
      timeout_critical_(500000000ns), // ns
      timeout_startup_(5000000000ns), // ns
      position_received_(true),
      should_exit_(false)
{
  // We are not retrieving the parameter list or the waypoint list from the vehicle
  // using this interface. So for now we limit the mission_item_speed_
  // to whatever the cruise speed is set.
  this->px4ParamsInit();
  mission_item_speed_ = px4_.param_mpc_xy_cruise;

  this->init();
}

AvoidanceNode::~AvoidanceNode() {}

void AvoidanceNode::init() {
  setSystemStatus(MAV_STATE::MAV_STATE_BOOT);

  
  avoidance_node_cmd = rclcpp::Node::make_shared("avoidance_node_cmd");
  cmdloop_timer_ = avoidance_node_cmd->create_wall_timer(cmdloop_dt_, [&](){});
  cmdloop_executor_.add_node(avoidance_node_cmd);

  // Start cmdloop thread
  cmdloop_thread = new std::thread([&]() {
    cmdloop_executor_.spin();
  });

  avoidance_node_status = rclcpp::Node::make_shared("avoidance_node_status");
  // This is a passthrough that replaces the usage of Mavlink Heartbeats
  telemetry_status_pub_ =
        avoidance_node_status->create_publisher<px4_msgs::msg::TelemetryStatus>("/TelemetryStatus_PubSubTopic", 1);
  statusloop_timer_ = avoidance_node_status->create_wall_timer(statusloop_dt_, [&](){ publishSystemStatus(); });
  statusloop_executor_.add_node(avoidance_node_status);

  // Start statusloop thread
  statusloop_thread = new std::thread([&]() {
    statusloop_executor_.spin();
  });
}

void AvoidanceNode::setSystemStatus(MAV_STATE state) {
    companion_state_ = state;
}

MAV_STATE AvoidanceNode::getSystemStatus() {
    return companion_state_;
}

void AvoidanceNode::publishSystemStatus() {
  // Publish companion process status as telemetry_status msg
  auto status_msg = px4_msgs::msg::TelemetryStatus();

  // TODO : TelemetryStatus.msg is totally changed in px4_msgs. It should be reflected.
  /*status_msg.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
  status_msg.heartbeat_time = status_msg.timestamp;
  status_msg.remote_system_id = 1;
  status_msg.remote_component_id = px4_msgs::msg::TelemetryStatus::COMPONENT_ID_OBSTACLE_AVOIDANCE;
  status_msg.remote_type = px4_msgs::msg::TelemetryStatus::MAV_TYPE_ONBOARD_CONTROLLER;
  status_msg.remote_system_status = (int)(this->getSystemStatus());
  status_msg.type = px4_msgs::msg::TelemetryStatus::LINK_TYPE_WIRE;*/

  telemetry_status_pub_->publish(status_msg);
}

void AvoidanceNode::checkFailsafe(rclcpp::Duration since_last_cloud, rclcpp::Duration since_start, bool& hover) {
  if (since_last_cloud > timeout_termination_ && since_start > timeout_termination_) {
    setSystemStatus(MAV_STATE::MAV_STATE_FLIGHT_TERMINATION);
    RCLCPP_WARN(avoidance_node_logger_, "\033[1;33m Planner abort: missing required data \n \033[0m");
  } else {
    if (since_last_cloud > timeout_critical_ && since_start > timeout_startup_) {
      if (position_received_) {
        hover = true;
        setSystemStatus(MAV_STATE::MAV_STATE_CRITICAL);
        std::string not_received = "";
      } else {
        RCLCPP_WARN(avoidance_node_logger_, "\033[1;33m Pointcloud timeout: No position received, no WP to output.... \n \033[0m");
      }
    } else {
      if (!hover) setSystemStatus(MAV_STATE::MAV_STATE_ACTIVE);
    }
  }
}

void AvoidanceNode::px4ParamsInit() {
  // inits PX4 parameters needed for model based trajectory planning manually
  // done while we don't have in interface that exposes the vehicle dynamics

  // clang-format off
  px4_.param_mpc_acc_down_max = 3.0f;
  px4_.param_mpc_acc_hor = 3.0f;
  px4_.param_acc_up_max = 4.0f;
  px4_.param_mpc_auto_mode = 1; // Deprecated
  px4_.param_mpc_jerk_min = 8.0f;
  px4_.param_mpc_jerk_max = 8.0f;
  px4_.param_mpc_land_speed = 0.7f;
  px4_.param_mpc_tko_speed = 1.5f;
  px4_.param_mpc_xy_cruise = 5.0f;
  px4_.param_mpc_z_vel_max_up = 1.0f;
  px4_.param_cp_dist = -1.0f;
  px4_.param_nav_acc_rad = 10.0;
  // clang-format on
}

}
