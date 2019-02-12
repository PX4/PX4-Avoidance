#ifndef AVOIDANCE_COMPANIONSTATUS_H
#define AVOIDANCE_COMPANIONSTATUS_H

#define MAV_COMPONENT_ID_AVOIDANCE 196

#include "ros/ros.h"
#include "mavros_msgs/CompanionProcessStatus.h"

namespace avoidance {

class CompanionStatus {
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Timer publoop_timer_;
  ros::Publisher companionstatus_pub_;

  double publish_rate_;

  int status_;

 public:
  CompanionStatus(const ros::NodeHandle& nh,  const ros::NodeHandle& nh_private, double publish_rate);
  virtual ~CompanionStatus();

  void PubloopCallback(const ros::TimerEvent& event);
  void setStatus(int status);
};
}

#endif  // AVOIDANCE_COMPANIONSTATUS_H