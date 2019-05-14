#include "avoidance/avoidance_node.h"

namespace avoidance {

AvoidanceNode::AvoidanceNode(const ros::NodeHandle& nh,
                                   const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), spin_dt_(0.1) {

  ros::TimerOptions timer_options(
      ros::Duration(spin_dt_),
      boost::bind(&AvoidanceNode::cmdLoopCallback, this, _1),
      &cmdloop_queue_);
  cmdloop_timer_ = nh_.createTimer(timer_options);

  cmdloop_spinner_.reset(new ros::AsyncSpinner(1, &cmdloop_queue_));
  cmdloop_spinner_->start();
}

AvoidanceNode::~AvoidanceNode() {
}

void AvoidanceNode::cmdLoopCallback(const ros::TimerEvent& event){

}


}