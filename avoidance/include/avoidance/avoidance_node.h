#ifndef AVOIDANCE_AVOIDANCE_NODE_H
#define AVOIDANCE_AVOIDANCE_NODE_H

#include "ros/ros.h"


namespace avoidance {

class AvoidanceNode {
 public:
  AvoidanceNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  ~AvoidanceNode();


 private:

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Timer cmdloop_timer_;


  double spin_dt_;

};
}
#endif  // AVOIDANCE_AVOIDANCE_NODE_H
