#ifndef AVOIDANCE_AVOIDANCE_NODE_H
#define AVOIDANCE_AVOIDANCE_NODE_H

#include "ros/ros.h"
#include "ros/callback_queue.h"

#include "avoidance/common.h"
#include "mavros_msgs/CompanionProcessStatus.h"


namespace avoidance {

class AvoidanceNode {
 public:
  AvoidanceNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  ~AvoidanceNode();


 private:

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher mavros_system_status_pub_;

  ros::Time start_time_;
  ros::Time last_wp_time_;
  ros::Time t_status_sent_;

  ros::Timer cmdloop_timer_, statusloop_timer_;
  ros::CallbackQueue cmdloop_queue_, statusloop_queue_;
  std::unique_ptr<ros::AsyncSpinner> cmdloop_spinner_;
  std::unique_ptr<ros::AsyncSpinner> statusloop_spinner_;

  MAV_STATE companion_state_ = MAV_STATE::MAV_STATE_STANDBY;

  double cmdloop_dt_, statusloop_dt_;

  void cmdLoopCallback(const ros::TimerEvent& event);
  void statusLoopCallback(const ros::TimerEvent& event);
  void publishSystemStatus();
  void setSystemStatus(MAV_STATE state);
  
};
}
#endif  // AVOIDANCE_AVOIDANCE_NODE_H
