#ifndef AVOIDANCE_AVOIDANCE_NODE_H
#define AVOIDANCE_AVOIDANCE_NODE_H

#include "ros/ros.h"
#include "ros/callback_queue.h"


namespace avoidance {

class AvoidanceNode {
 public:
  AvoidanceNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  ~AvoidanceNode();


 private:

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Timer cmdloop_timer_, statusloop_timer_;
  ros::CallbackQueue cmdloop_queue_, statusloop_queue_;
  std::unique_ptr<ros::AsyncSpinner> cmdloop_spinner_;
  std::unique_ptr<ros::AsyncSpinner> statusloop_spinner_;

  double cmdloop_dt_, statusloop_dt_;

  void cmdLoopCallback(const ros::TimerEvent& event);
  void statusLoopCallback(const ros::TimerEvent& event);
  
};
}
#endif  // AVOIDANCE_AVOIDANCE_NODE_H
