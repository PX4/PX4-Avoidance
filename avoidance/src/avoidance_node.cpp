#include "avoidance/avoidance_node.h"

namespace avoidance {

AvoidanceNode::AvoidanceNode(const ros::NodeHandle& nh,
                                   const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), cmdloop_dt_(0.1), statusloop_dt_(0.2) {


  mavros_system_status_pub_ =
      nh_.advertise<mavros_msgs::CompanionProcessStatus>(
          "/mavros/companion_process/status", 1);

  ros::TimerOptions cmdlooptimer_options(
      ros::Duration(cmdloop_dt_),
      boost::bind(&AvoidanceNode::cmdLoopCallback, this, _1),
      &cmdloop_queue_);
  cmdloop_timer_ = nh_.createTimer(cmdlooptimer_options);

  ros::TimerOptions statuslooptimer_options(
      ros::Duration(statusloop_dt_),
      boost::bind(&AvoidanceNode::statusLoopCallback, this, _1),
      &statusloop_queue_);
  statusloop_timer_ = nh_.createTimer(statuslooptimer_options);

  setSystemStatus(MAV_STATE::MAV_STATE_BOOT);

  cmdloop_spinner_.reset(new ros::AsyncSpinner(1, &cmdloop_queue_));
  cmdloop_spinner_->start();
  statusloop_spinner_.reset(new ros::AsyncSpinner(1, &statusloop_queue_));
  statusloop_spinner_->start();

  position_received_ = true;

  timeout_termination_ = 15;
  timeout_critical_ = 0.5;
  timeout_startup_ = 5.0;
}

AvoidanceNode::~AvoidanceNode() {
}

void AvoidanceNode::cmdLoopCallback(const ros::TimerEvent& event){

}

void AvoidanceNode::statusLoopCallback(const ros::TimerEvent& event){
  
  publishSystemStatus();
}

void AvoidanceNode::setSystemStatus(MAV_STATE state) {
  companion_state_ = state;
}

// Publish companion process status
void AvoidanceNode::publishSystemStatus() {
  mavros_msgs::CompanionProcessStatus status_msg;
  status_msg.header.stamp = ros::Time::now();
  status_msg.component = 196;  // MAV_COMPONENT_ID_AVOIDANCE
  status_msg.state = (int)companion_state_;

  mavros_system_status_pub_.publish(status_msg);
  t_status_sent_ = ros::Time::now();
}

void AvoidanceNode::checkFailsafe(ros::Duration since_last_cloud,
                                    ros::Duration since_start, bool& hover) {

  ros::Duration timeout_termination = ros::Duration(timeout_termination_);
  ros::Duration timeout_critical = ros::Duration(timeout_critical_);
  ros::Duration timeout_startup = ros::Duration(timeout_startup_);

  if (since_last_cloud > timeout_termination &&
      since_start > timeout_termination) {
    setSystemStatus(MAV_STATE::MAV_STATE_FLIGHT_TERMINATION);
    ROS_WARN("\033[1;33m Planner abort: missing required data \n \033[0m");
  } else {
    if (since_last_cloud > timeout_critical && since_start > timeout_startup) {
      if (position_received_) {
        hover = true;
        setSystemStatus(MAV_STATE::MAV_STATE_CRITICAL);
        std::string not_received = "";
      } else {
        ROS_WARN(
            "\033[1;33m Pointcloud timeout: No position received, no WP to "
            "output.... \n \033[0m");
      }
    } else {
      if (!hover) setSystemStatus(MAV_STATE::MAV_STATE_ACTIVE);
    }
  }
}
}