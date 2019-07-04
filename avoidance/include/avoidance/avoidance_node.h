#ifndef AVOIDANCE_AVOIDANCE_NODE_H
#define AVOIDANCE_AVOIDANCE_NODE_H

#include "ros/callback_queue.h"
#include "ros/ros.h"

#include "avoidance/common.h"
#include "mavros_msgs/CompanionProcessStatus.h"

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
  void checkFailsafe(ros::Duration since_last_cloud, ros::Duration since_start,
                     bool& hover);
  void setSystemStatus(MAV_STATE state);
  MAV_STATE getSystemStatus();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher mavros_system_status_pub_;

  ros::Timer cmdloop_timer_, statusloop_timer_;
  ros::CallbackQueue cmdloop_queue_, statusloop_queue_;
  std::unique_ptr<ros::AsyncSpinner> cmdloop_spinner_;
  std::unique_ptr<ros::AsyncSpinner> statusloop_spinner_;

  MAV_STATE companion_state_ = MAV_STATE::MAV_STATE_STANDBY;

  double cmdloop_dt_, statusloop_dt_;
  double timeout_termination_;
  double timeout_critical_;
  double timeout_startup_;

  bool position_received_;

  void cmdLoopCallback(const ros::TimerEvent& event);
  void statusLoopCallback(const ros::TimerEvent& event);
  void publishSystemStatus();
};
}
#endif  // AVOIDANCE_AVOIDANCE_NODE_H
