#ifndef GLOBAL_PLANNER_PATH_HANDLER_NODE_H
#define GLOBAL_PLANNER_PATH_HANDLER_NODE_H

#include <math.h>
#include <map>
#include <vector>

#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/Trajectory.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <global_planner/PathHandlerNodeConfig.h>
#include <global_planner/PathWithRiskMsg.h>
#include "global_planner/common.h"
#include "global_planner/common_ros.h"

#include <global_planner/ThreePointMsg.h>

namespace global_planner {

class PathHandlerNode {
 public:
  PathHandlerNode();
  ~PathHandlerNode();

 private:
  ros::NodeHandle nh_;
  dynamic_reconfigure::Server<global_planner::PathHandlerNodeConfig> server_;

  // Parameters (Rosparam)
  geometry_msgs::Point start_pos_;
  double start_yaw_;
  // Parameters (Dynamic Reconfiguration)
  bool three_point_mode_;
  bool ignore_path_messages_;
  double min_speed_;
  double max_speed_;
  double three_point_speed_;
  double direct_goal_alt_;

  double speed_ = min_speed_;
  geometry_msgs::PoseStamped current_goal_;
  geometry_msgs::PoseStamped last_goal_;
  geometry_msgs::PoseStamped last_pos_;

  std::vector<geometry_msgs::PoseStamped> path_;
  std::map<tf::Vector3, double> path_risk_;

  ros::Subscriber direct_goal_sub_;
  ros::Subscriber path_sub_;
  ros::Subscriber path_with_risk_sub_;
  ros::Subscriber ground_truth_sub_;

  ros::Publisher mavros_waypoint_publisher_;
  ros::Publisher current_waypoint_publisher_;
  ros::Publisher three_point_path_publisher_;
  ros::Publisher three_point_msg_publisher_;
  ros::Publisher avoidance_triplet_msg_publisher_;
  ros::Publisher mavros_obstacle_free_path_pub_;

  tf::TransformListener listener_;

  // Methods
  void readParams();
  bool shouldPublishThreePoints();
  bool isCloseToGoal();
  double getRiskOfCurve(const std::vector<geometry_msgs::PoseStamped>& poses);
  void setCurrentPath(const std::vector<geometry_msgs::PoseStamped>& poses);
  // Callbacks
  void dynamicReconfigureCallback(global_planner::PathHandlerNodeConfig& config,
                                  uint32_t level);
  void receiveDirectGoal(const geometry_msgs::PoseWithCovarianceStamped& msg);
  void receivePath(const nav_msgs::Path& msg);
  void receivePathWithRisk(const PathWithRiskMsg& msg);
  void positionCallback(const geometry_msgs::PoseStamped& pose_msg);
  // Publishers
  void transformPoseToObstacleAvoidance(mavros_msgs::Trajectory &obst_avoid, geometry_msgs::PoseStamped pose);
  void publishSetpoint();
  void publishThreePointMsg();
};

}  // namespace global_planner

#endif  // GLOBAL_PLANNER_PATH_HANDLER_NODE_H
