#ifndef LOCAL_PLANNER_COMMON_ROS_H_
#define LOCAL_PLANNER_COMMON_ROS_H_

#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <math.h>  // sqrt
#include <nav_msgs/Path.h>
#include <std_msgs/ColorRGBA.h>
#include <tf/transform_listener.h>  // getYaw createQuaternionMsgFromYaw
#include <visualization_msgs/Marker.h>

// This file contains general functions which have some Ros dependancy

namespace avoidance {

geometry_msgs::TwistStamped transformTwistMsg(
    const tf::TransformListener& listener, const std::string& target_frame,
    const std::string& fixed_frame, const geometry_msgs::TwistStamped& msg) {
  auto transformed_msg = msg;
  geometry_msgs::Vector3Stamped before;
  before.vector = msg.twist.linear;
  before.header = msg.header;
  geometry_msgs::Vector3Stamped after;
  listener.transformVector(target_frame, ros::Time(0), before, fixed_frame,
                           after);
  transformed_msg.twist.linear = after.vector;
  return transformed_msg;
}

// Returns true if msg1 and msg2 have both the same altitude and orientation
bool hasSameYawAndAltitude(const geometry_msgs::Pose& msg1,
                           const geometry_msgs::Pose& msg2) {
  return msg1.orientation.z == msg2.orientation.z &&
         msg1.orientation.w == msg2.orientation.w &&
         msg1.position.z == msg2.position.z;
}

}  // namespace avoidance

#endif /* LOCAL_PLANNER_COMMON_ROS_H_ */
