#ifndef AVOIDANCE_AVOIDANCE_NODE_H
#define AVOIDANCE_AVOIDANCE_NODE_H

#include "ros/callback_queue.h"
#include "ros/ros.h"

#include <mavros_msgs/Param.h>
#include <mavros_msgs/ParamGet.h>
#include "avoidance/common.h"
#include "mavros_msgs/CompanionProcessStatus.h"

#include <thread>

namespace avoidance {

class AvoidanceNode {
 public:
  AvoidanceNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
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
  void checkFailsafe(ros::Duration since_last_cloud, ros::Duration since_start, bool& hover);

  ModelParameters getPX4Parameters() const { return px4_; }

  /**
  * @brief     polls PX4 Firmware paramters every 30 seconds
  **/
  void checkPx4Parameters();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher mavros_system_status_pub_;

  ros::Subscriber px4_param_sub_;

  ros::ServiceClient get_px4_param_client_;

  ros::Timer cmdloop_timer_, statusloop_timer_;
  ros::CallbackQueue cmdloop_queue_, statusloop_queue_;
  std::unique_ptr<ros::AsyncSpinner> cmdloop_spinner_;
  std::unique_ptr<ros::AsyncSpinner> statusloop_spinner_;

  MAV_STATE companion_state_ = MAV_STATE::MAV_STATE_STANDBY;

  ModelParameters px4_;  // PX4 Firmware paramters

  std::thread worker_;

  double cmdloop_dt_, statusloop_dt_;
  double timeout_termination_;
  double timeout_critical_;
  double timeout_startup_;

  bool position_received_;
  bool should_exit_;

  void cmdLoopCallback(const ros::TimerEvent& event);
  void statusLoopCallback(const ros::TimerEvent& event);
  void publishSystemStatus();
  void setSystemStatus(MAV_STATE state);
  /**
  * @brief     polls PX4 Firmware paramters every 30 seconds
  **/
  void px4ParamsCallback(const mavros_msgs::Param& msg);
};
}
#endif  // AVOIDANCE_AVOIDANCE_NODE_H
