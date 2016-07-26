#ifndef GLOBAL_PLANNER_GLOBAL_PLANNER_NODE_H
#define GLOBAL_PLANNER_GLOBAL_PLANNER_NODE_H

#include <boost/bind.hpp>
// #include <Eigen/Eigen>
#include <math.h>           // abs floor
#include <set>
#include <stdio.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
// #include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/transforms.h> // transformPointCloud
#include <pcl_conversions/pcl_conversions.h>  // fromROSMsg
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h> // getYaw createQuaternionMsgFromYaw  TransformListener
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>

#include "avoidance/cell.h"
#include "avoidance/common.h"
#include "avoidance/global_planner.h"



namespace avoidance {

class GlobalPlannerNode {
 public:
  // TODO: Deque instead of vector
  std::vector<GoalCell> waypoints_;  // Intermediate goals, from file, mavros mission or intermediate goals
  GlobalPlannerNode();
  ~GlobalPlannerNode();

 private:
  GlobalPlanner global_planner_;
  nav_msgs::Path actual_path_;

  int num_octomap_msg_ = 0;
  int num_pos_msg_ = 0;

  // Subscribers
  ros::Subscriber waypoint_sub_;
  ros::Subscriber octomap_sub_;
  ros::Subscriber octomap_full_sub_;
  ros::Subscriber ground_truth_sub_;
  ros::Subscriber velocity_sub_;
  ros::Subscriber clicked_point_sub_;
  ros::Subscriber laser_sensor_sub_;
  ros::Subscriber depth_camera_sub_;

  // Publishers
  ros::Publisher global_path_pub_;
  ros::Publisher global_temp_path_pub_;
  ros::Publisher actual_path_pub_;
  ros::Publisher explored_cells_pub_;
  ros::Publisher global_goal_pub_;
  ros::Publisher global_temp_goal_pub_;

  tf::TransformListener listener_;

  void SetNewGoal(const GoalCell & goal);
  void PopNextGoal();
  void PlanPath();
  void SetIntermediateGoal();

  void VelocityCallback(const geometry_msgs::TwistStamped & msg);
  void PositionCallback(const geometry_msgs::PoseStamped & msg);
  void ClickedPointCallback(const geometry_msgs::PointStamped & msg);
  void LaserSensorCallback(const sensor_msgs::LaserScan & msg);
  void OctomapFullCallback(const octomap_msgs::Octomap & msg);
  void DepthCameraCallback(const sensor_msgs::PointCloud2 & msg);

  void PublishGoal(const GoalCell & goal);
  void PublishPath();
  void PublishExploredCells();
};

} // namespace avoidance

#endif // GLOBAL_PLANNER_GLOBAL_PLANNER_NODE_H
