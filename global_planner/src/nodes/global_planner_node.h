#ifndef GLOBAL_PLANNER_GLOBAL_PLANNER_NODE_H
#define GLOBAL_PLANNER_GLOBAL_PLANNER_NODE_H

#include <math.h>
#include <stdio.h>
#include <boost/bind.hpp>
#include <set>
#include <string>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/Trajectory.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/ColorRGBA.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include "global_planner/PathWithRiskMsg.h"
#include "global_planner/analysis.h"
#include "global_planner/cell.h"
#include "global_planner/common.h"
#include "global_planner/common_ros.h"
#include "global_planner/global_planner.h"
#include "global_planner/search_tools.h"
#include "global_planner/visitor.h"

namespace global_planner {

class GlobalPlannerNode {
 public:
  // TODO: Deque instead of vector
  GlobalPlanner global_planner_;
  std::vector<GoalCell> waypoints_;  // Intermediate goals, from file, mavros
                                     // mission or intermediate goals
  GlobalPlannerNode();
  ~GlobalPlannerNode();

 private:
  ros::NodeHandle nh_;
  dynamic_reconfigure::Server<global_planner::GlobalPlannerNodeConfig> server_;

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
  ros::Subscriber fcu_input_sub_;

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

  void setNewGoal(const GoalCell& goal);
  void popNextGoal();
  void planPath();
  void setIntermediateGoal();

  void dynamicReconfigureCallback(
      global_planner::GlobalPlannerNodeConfig& config, uint32_t level);
  void velocityCallback(const geometry_msgs::TwistStamped& msg);
  void positionCallback(const geometry_msgs::PoseStamped& msg);
  void clickedPointCallback(const geometry_msgs::PointStamped& msg);
  void threePointCallback(const nav_msgs::Path& msg);
  void moveBaseSimpleCallback(const geometry_msgs::PoseStamped& msg);
  void laserSensorCallback(const sensor_msgs::LaserScan& msg);
  void octomapFullCallback(const octomap_msgs::Octomap& msg);
  void depthCameraCallback(const sensor_msgs::PointCloud2& msg);
  void fcuInputGoalCallback(const mavros_msgs::Trajectory& msg);
  void publishGoal(const GoalCell& goal);
  void publishPath();
  void publishExploredCells();

  void printPointInfo(double x, double y, double z);
};

}  // namespace global_planner

#endif  // GLOBAL_PLANNER_GLOBAL_PLANNER_NODE_H
