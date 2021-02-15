#ifndef AVOIDANCE_AVOIDANCE_NODE_H
#define AVOIDANCE_AVOIDANCE_NODE_H

#include <rclcpp/rclcpp.hpp>

#include "avoidance/common.h"
#include <px4_msgs/msg/telemetry_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <chrono>

namespace avoidance {

using std::placeholders::_1;

class AvoidanceNode {
 public:
  AvoidanceNode();
  ~AvoidanceNode();
  /**
  * @brief      check healthiness of the avoidance system to trigger failsafe in
  *             the FCU
  * @param[in]  since_last_cloud, time elapsed since the last waypoint was
  *             published to the FCU
  * @param[in]  since_start, time elapsed since staring the node
  * @param[out] planner_is_healthy, true if the planner is running without
  *errors
  * @param[out] hover, true if the vehicle is hovering
  **/
  void checkFailsafe(rclcpp::Duration since_last_cloud, rclcpp::Duration since_start, bool& hover);

  MAV_STATE getSystemStatus();

  void setSystemStatus(MAV_STATE state);

  void init();

  void px4ParamsInit();

 private:
  // telemetry_status Publisher
  rclcpp::Publisher<px4_msgs::msg::TelemetryStatus>::SharedPtr telemetry_status_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_sub_;

  rclcpp::executors::MultiThreadedExecutor cmdloop_executor_;
  rclcpp::executors::MultiThreadedExecutor statusloop_executor_;
  rclcpp::TimerBase::SharedPtr cmdloop_timer_;
  rclcpp::TimerBase::SharedPtr statusloop_timer_;

  std::thread* statusloop_thread;
  std::thread* cmdloop_thread;
  
  rclcpp::Node::SharedPtr avoidance_node_status;
  rclcpp::Node::SharedPtr avoidance_node_cmd;

  MAV_STATE companion_state_ = MAV_STATE::MAV_STATE_STANDBY;

  // PX4 Firmware parameters
  ModelParameters px4_;

  std::chrono::milliseconds cmdloop_dt_;
  std::chrono::milliseconds statusloop_dt_;

  rclcpp::Duration timeout_termination_;
  rclcpp::Duration timeout_critical_;
  rclcpp::Duration timeout_startup_;

  int agent_number_ = 1;
  bool position_received_;
  bool should_exit_;

  float mission_item_speed_;

  void publishSystemStatus();
  
  rclcpp::Logger avoidance_node_logger_ = rclcpp::get_logger("avoidance_node");

};
}
#endif  // AVOIDANCE_AVOIDANCE_NODE_H