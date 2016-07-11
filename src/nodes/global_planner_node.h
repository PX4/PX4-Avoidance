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
  std::vector<Cell> fileGoals;  // If a goal file is given, the intermediate goals are stored in fileGoals
  GlobalPlannerNode();
  ~GlobalPlannerNode();

 private:
  GlobalPlanner global_planner;
  geometry_msgs::Point goalPoint;
  nav_msgs::Path actualPath;

  int numOctomapMessages = 0;
  int numPositionMessages = 0;

  // Subscribers
  ros::Subscriber cmd_waypoint_sub_;
  ros::Subscriber cmd_octomap_sub_;
  ros::Subscriber cmd_octomap_full_sub_;
  ros::Subscriber cmd_ground_truth_sub_;
  ros::Subscriber velocity_sub_;
  ros::Subscriber cmd_clicked_point_sub_;
  ros::Subscriber laser_sensor_sub_;
  ros::Subscriber depth_camera_sub_;

  // Publishers
  ros::Publisher cmd_global_path_pub_;
  ros::Publisher cmd_actual_path_pub_;
  ros::Publisher cmd_explored_cells_pub_;
  ros::Publisher cmd_clicked_point_pub_;

  tf::TransformListener listener;

  void SetNewGoal(Cell goal);
  void VelocityCallback(const geometry_msgs::TwistStamped& msg);
  void PositionCallback(const geometry_msgs::PoseStamped& msg);
  void ClickedPointCallback(const geometry_msgs::PointStamped& msg);
  void LaserSensorCallback(const sensor_msgs::LaserScan& msg);
  void OctomapCallback(const visualization_msgs::MarkerArray& msg);
  void OctomapFullCallback(const octomap_msgs::Octomap& msg);
  void DepthCameraCallback(const sensor_msgs::PointCloud2& msg);

  void PlanPath();

  void PublishPath();
  void PublishExploredCells();
};

} // namespace avoidance

#endif // GLOBAL_PLANNER_GLOBAL_PLANNER_NODE_H
