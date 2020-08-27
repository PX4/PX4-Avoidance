#ifndef GLOBAL_PLANNER_GLOBAL_PLANNER_NODE_H
#define GLOBAL_PLANNER_GLOBAL_PLANNER_NODE_H

#include <math.h>
#include <stdio.h>
#include <boost/bind.hpp>
#include <mutex>
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

#include "avoidance/avoidance_node.h"
#include "global_planner/global_planner.h"

#ifndef DISABLE_SIMULATION
#include <avoidance/rviz_world_loader.h>
#endif

namespace global_planner {

struct cameraData {
  ros::Subscriber pointcloud_sub_;
};

class GlobalPlannerNode {
 public:
  // TODO: Deque instead of vector
  GlobalPlanner global_planner_;
  std::vector<GoalCell> waypoints_;  // Intermediate goals, from file, mavros
                                     // mission or intermediate goals
  GlobalPlannerNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  ~GlobalPlannerNode();

 private:
  std::mutex mutex_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Subscribers
  ros::Subscriber octomap_sub_;
  ros::Subscriber octomap_full_sub_;
  ros::Subscriber ground_truth_sub_;
  ros::Subscriber velocity_sub_;
  ros::Subscriber clicked_point_sub_;
  ros::Subscriber move_base_simple_sub_;
  ros::Subscriber laser_sensor_sub_;
  ros::Subscriber fcu_input_sub_;

  // Publishers
  ros::Publisher global_temp_path_pub_;
  ros::Publisher smooth_path_pub_;
  ros::Publisher actual_path_pub_;
  ros::Publisher explored_cells_pub_;
  ros::Publisher global_goal_pub_;
  ros::Publisher global_temp_goal_pub_;
  ros::Publisher mavros_obstacle_free_path_pub_;
  ros::Publisher mavros_waypoint_publisher_;
  ros::Publisher current_waypoint_publisher_;
  ros::Publisher pointcloud_pub_;

  ros::Time start_time_;
  ros::Time last_wp_time_;

  ros::Timer cmdloop_timer_;
  ros::Timer plannerloop_timer_;
  ros::CallbackQueue cmdloop_queue_;
  ros::CallbackQueue plannerloop_queue_;
  std::unique_ptr<ros::AsyncSpinner> cmdloop_spinner_;
  std::unique_ptr<ros::AsyncSpinner> plannerloop_spinner_;

  tf::TransformListener listener_;
  dynamic_reconfigure::Server<global_planner::GlobalPlannerNodeConfig> server_;

  nav_msgs::Path actual_path_;
  geometry_msgs::Point start_pos_;
  geometry_msgs::PoseStamped current_goal_;
  geometry_msgs::PoseStamped last_goal_;
  geometry_msgs::PoseStamped last_pos_;

  std::vector<geometry_msgs::PoseStamped> last_clicked_points;
  std::vector<geometry_msgs::PoseStamped> path_;
  std::vector<cameraData> cameras_;

  int num_octomap_msg_ = 0;
  int num_pos_msg_ = 0;
  double cmdloop_dt_;
  double plannerloop_dt_;
  double mapupdate_dt_;
  double speed_;
  double start_yaw_;
  bool position_received_;
  std::string frame_id_;
  std::string camera_frame_id_;

  // Dynamic Reconfiguration
  double clicked_goal_alt_;
  double clicked_goal_radius_;
  bool hover_;
  int simplify_iterations_;
  double simplify_margin_;

  avoidance::AvoidanceNode avoidance_node_;
#ifndef DISABLE_SIMULATION
  std::unique_ptr<avoidance::WorldVisualizer> world_visualizer_;
#endif
  void readParams();
  void initializeCameraSubscribers(std::vector<std::string>& camera_topics);
  void receivePath(const nav_msgs::Path& msg);
  void setNewGoal(const GoalCell& goal);
  void popNextGoal();
  void planPath();
  void setIntermediateGoal();
  bool isCloseToGoal();
  void setCurrentPath(const std::vector<geometry_msgs::PoseStamped>& poses);
  void dynamicReconfigureCallback(global_planner::GlobalPlannerNodeConfig& config, uint32_t level);
  void velocityCallback(const geometry_msgs::TwistStamped& msg);
  void positionCallback(const geometry_msgs::PoseStamped& msg);
  void clickedPointCallback(const geometry_msgs::PointStamped& msg);
  void moveBaseSimpleCallback(const geometry_msgs::PoseStamped& msg);
  void octomapFullCallback(const octomap_msgs::Octomap& msg);
  void depthCameraCallback(const sensor_msgs::PointCloud2& msg);
  void fcuInputGoalCallback(const mavros_msgs::Trajectory& msg);
  void cmdLoopCallback(const ros::TimerEvent& event);
  void plannerLoopCallback(const ros::TimerEvent& event);
  void publishGoal(const GoalCell& goal);
  void publishPath();
  void publishSetpoint();
  void printPointInfo(double x, double y, double z);
};

}  // namespace global_planner

#endif  // GLOBAL_PLANNER_GLOBAL_PLANNER_NODE_H
