#ifndef GLOBAL_PLANNER_PATH_HANDLER_NODE_H
#define GLOBAL_PLANNER_PATH_HANDLER_NODE_H

#include <math.h> // floor
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>  // getYaw
#include <ros/ros.h>

#include "avoidance/common.h" // hasSameYawAndAltitude

namespace avoidance {

class PathHandlerNode {
 public:
  PathHandlerNode();
  ~PathHandlerNode();

 private:
  geometry_msgs::PoseStamped currentGoal;
  geometry_msgs::PoseStamped lastPos;
  double speed = 1.0;
  double maxSpeed = 1.5;

  std::vector<geometry_msgs::PoseStamped> path;

  ros::Subscriber trajectory_sub_;
  ros::Subscriber ground_truth_sub_;

  ros::Publisher mavros_waypoint_publisher_;
  ros::Publisher current_waypoint_publisher_;
  ros::Publisher three_point_path_publisher_;

  tf::TransformListener listener_;

  void ReceiveMessage(const geometry_msgs::PoseStamped & pose_msg);
  void ReceivePath(const nav_msgs::Path & msg);
  void PositionCallback(const geometry_msgs::PoseStamped & pose_msg);
};

} // namespace avoidance

#endif // GLOBAL_PLANNER_PATH_HANDLER_NODE_H
