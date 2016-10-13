#ifndef GLOBAL_PLANNER_PATH_HANDLER_NODE_H
#define GLOBAL_PLANNER_PATH_HANDLER_NODE_H

#include <math.h> // floor
#include <map>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>  // getYaw
#include <ros/ros.h>

#include "avoidance/common.h" // hasSameYawAndAltitude
#include "avoidance/PathWithRiskMsg.h"
#include "avoidance/ThreePointMsg.h"


namespace avoidance {

class PathHandlerNode {
 public:
  PathHandlerNode();
  ~PathHandlerNode();

 private:
  geometry_msgs::PoseStamped current_goal_;
  geometry_msgs::PoseStamped last_goal_;
  geometry_msgs::PoseStamped last_pos_;
  double min_speed_ = 2.0;
  double max_speed_ = 2.1;
  double speed_ = min_speed_;

  std::vector<geometry_msgs::PoseStamped> path_;
  std::map<tf::Vector3, double> path_risk_;

  ros::Subscriber path_sub_;
  ros::Subscriber path_with_risk_sub_;
  ros::Subscriber ground_truth_sub_;

  ros::Publisher mavros_waypoint_publisher_;
  ros::Publisher current_waypoint_publisher_;
  ros::Publisher three_point_path_publisher_;
  ros::Publisher three_point_msg_publisher_;

  tf::TransformListener listener_;

  void receiveMessage(const geometry_msgs::PoseStamped & pose_msg);
  void receivePath(const nav_msgs::Path & msg);
  void receivePathWithRisk(const PathWithRiskMsg & msg);
  void positionCallback(const geometry_msgs::PoseStamped & pose_msg);
  void setCurrentPath(const std::vector<geometry_msgs::PoseStamped> & poses);
  void publishSetpoint();
  void publishThreePointMsg();
  double getRiskOfCurve(const std::vector<geometry_msgs::PoseStamped> & poses);

};

} // namespace avoidance

#endif // GLOBAL_PLANNER_PATH_HANDLER_NODE_H
