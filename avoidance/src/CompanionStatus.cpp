#include "avoidance/CompanionStatus.h"

namespace avoidance {

    CompanionStatus::CompanionStatus(const ros::NodeHandle& nh,
            const ros::NodeHandle& nh_private, double publish_rate)
            : nh_(nh), nh_private_(nh_private), publish_rate_(publish_rate) {

        companionstatus_pub_ = nh_.advertise<mavros_msgs::CompanionProcessStatus>("/mavros/companion_process/status", 1);
        publoop_timer_ = nh_.createTimer(ros::Duration(publish_rate), &CompanionStatus::PubloopCallback, this); // Define timer for constant loop rate

    }

    CompanionStatus::~CompanionStatus() {}

    void CompanionStatus::PubloopCallback(const ros::TimerEvent& event) {
      mavros_msgs::CompanionProcessStatus status_msg;

      status_msg.header.stamp = ros::Time::now();

      status_msg.component = MAV_COMPONENT_ID_AVOIDANCE;
      status_msg.state = status_;

      companionstatus_pub_.publish(status_msg);
    }

    void CompanionStatus::setStatus(int status) {
      status_ = status;
    }
}