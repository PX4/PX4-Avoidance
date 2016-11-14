#ifndef GLOBAL_PLANNER_GLOBAL_PLANNER_NODE_H
#define GLOBAL_PLANNER_GLOBAL_PLANNER_NODE_H

#include <boost/bind.hpp>
// #include <Eigen/Eigen>
#include <math.h>           // abs floor
#include <set>
#include <stdio.h>
#include <string>

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
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>

#include "avoidance/analysis.h"
#include "avoidance/cell.h"
#include "avoidance/common.h"
#include "avoidance/global_planner.h"
#include "avoidance/search_tools.h"
#include "avoidance/visitor.h"
#include "avoidance/PathWithRiskMsg.h"




namespace avoidance {

class GlobalPlannerNode {
 public:
  // TODO: Deque instead of vector
  GlobalPlanner global_planner_;
  std::vector<GoalCell> waypoints_;  // Intermediate goals, from file, mavros mission or intermediate goals
  GlobalPlannerNode();
  ~GlobalPlannerNode();

 private:
  ros::NodeHandle nh_;
  dynamic_reconfigure::Server<avoidance::GlobalPlannerNodeConfig> server_;
  
  nav_msgs::Path actual_path_;

  int num_octomap_msg_ = 0;
  int num_pos_msg_ = 0;
  std::vector<geometry_msgs::PoseStamped> last_clicked_points;

  // Dynamic Reconfiguration
  double clicked_goal_alt_;
  double clicked_goal_radius_;
  int simplify_iterations_;
  double simplify_margin_;

  // Subscribers
  ros::Subscriber octomap_sub_;
  ros::Subscriber octomap_full_sub_;
  ros::Subscriber ground_truth_sub_;
  ros::Subscriber velocity_sub_;
  ros::Subscriber clicked_point_sub_;
  ros::Subscriber three_point_sub_;
  ros::Subscriber move_base_simple_sub_;
  ros::Subscriber laser_sensor_sub_;
  ros::Subscriber depth_camera_sub_;

  // Publishers
  ros::Publisher three_points_pub_;
  ros::Publisher three_points_smooth_pub_;
  ros::Publisher three_points_revised_pub_;
  ros::Publisher global_path_pub_;
  ros::Publisher global_temp_path_pub_;
  ros::Publisher smooth_path_pub_;
  ros::Publisher actual_path_pub_;
  ros::Publisher explored_cells_pub_;
  ros::Publisher global_goal_pub_;
  ros::Publisher global_temp_goal_pub_;

  tf::TransformListener listener_;

  void readParams();
  
  void setNewGoal(const GoalCell & goal);
  void popNextGoal();
  void planPath();
  void setIntermediateGoal();

  void dynamicReconfigureCallback(avoidance::GlobalPlannerNodeConfig & config, uint32_t level);
  void velocityCallback(const geometry_msgs::TwistStamped & msg);
  void positionCallback(const geometry_msgs::PoseStamped & msg);
  void clickedPointCallback(const geometry_msgs::PointStamped & msg);
  void threePointCallback(const nav_msgs::Path & msg);
  void moveBaseSimpleCallback(const geometry_msgs::PoseStamped & msg);
  void laserSensorCallback(const sensor_msgs::LaserScan & msg);
  void octomapFullCallback(const octomap_msgs::Octomap & msg);
  void depthCameraCallback(const sensor_msgs::PointCloud2 & msg);

  void publishGoal(const GoalCell & goal);
  void publishPath();
  void publishExploredCells();

  void printPointInfo(double x, double y, double z);

};

} // namespace avoidance

#endif // GLOBAL_PLANNER_GLOBAL_PLANNER_NODE_H
