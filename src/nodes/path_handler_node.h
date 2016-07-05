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
  geometry_msgs::PoseStamped last_pos;
  double speed = 1.0;
  double maxSpeed = 1.5;

  std::vector<geometry_msgs::PoseStamped> path;

  ros::Subscriber cmd_trajectory_sub_;
  ros::Subscriber cmd_ground_truth_sub_;

  ros::Publisher mavros_waypoint_publisher;
  ros::Publisher current_waypoint_publisher;
  ros::Publisher three_point_path_publisher;

  void ReceiveMessage(const geometry_msgs::PoseStamped& pose_msg);
  void ReceivePath(const nav_msgs::Path& msg);
  void PositionCallback(const geometry_msgs::PoseStamped& pose_msg);
};

} // namespace avoidance

#endif // GLOBAL_PLANNER_PATH_HANDLER_NODE_H
