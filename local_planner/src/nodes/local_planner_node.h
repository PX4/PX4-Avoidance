#ifndef LOCAL_PLANNER_LOCAL_PLANNER_NODE_H
#define LOCAL_PLANNER_LOCAL_PLANNER_NODE_H

#include <math.h>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <atomic>
#include <condition_variable>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Trajectory.h>
#include <nav_msgs/GridCells.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>  // fromROSMsg
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>  // transformPointCloud
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Core>
#include <boost/bind.hpp>
#include "std_msgs/String.h"

#include "avoidance/common_ros.h"
#include "local_planner.h"
#include "planner_functions.h"
#include "waypoint_generator.h"

class LocalPlannerNode {
 public:
  LocalPlannerNode();
  ~LocalPlannerNode();

  bool never_run_ = true;
  bool position_received_ = false;

  std::atomic<bool> should_exit_{false};

  double curr_yaw_;

  ros::CallbackQueue pointcloud_queue_;
  ros::CallbackQueue main_queue_;

  std::vector<geometry_msgs::Point> path_node_positions_;
  geometry_msgs::PoseStamped hover_point_;
  geometry_msgs::PoseStamped newest_pose_;
  sensor_msgs::PointCloud2 newest_point_cloud_;
  geometry_msgs::PoseStamped goal_msg_;

  const ros::Duration pointcloud_timeout_hover_ = ros::Duration(0.4);
  const ros::Duration pointcloud_timeout_land_ = ros::Duration(10);

  ros::Time last_wp_time_;

  LocalPlanner local_planner_;
  WaypointGenerator wp_generator_;

  ros::Publisher log_name_pub_;
  ros::Publisher current_waypoint_pub_;
  ros::Publisher mavros_pos_setpoint_pub_;
  ros::Publisher mavros_vel_setpoint_pub_;
  ros::Publisher mavros_obstacle_free_path_pub_;
  ros::Publisher mavros_obstacle_distance_pub_;
  ros::Publisher waypoint_pub_;
  ros::ServiceClient mavros_set_mode_client_;
  tf::TransformListener tf_listener_;

  std::mutex running_mutex_; ///< guard against concurrent access to input & output data (point cloud, position, ...)

  std::mutex data_ready_mutex_;
  bool data_ready_ = false;
  std::condition_variable data_ready_cv_;

  void publishSetpoint(const geometry_msgs::Twist& wp,
                       waypoint_choice& waypoint_type);
  void threadFunction();
  void getInterimWaypoint(geometry_msgs::PoseStamped &wp,
                          geometry_msgs::Twist &wp_vel);
  bool canUpdatePlannerInfo();
  void updatePlannerInfo();
  void transformPoseToTrajectory(mavros_msgs::Trajectory &obst_avoid,
                                 geometry_msgs::PoseStamped pose);
  void transformVelocityToTrajectory(mavros_msgs::Trajectory &obst_avoid,
                                     geometry_msgs::Twist vel);
  void fillUnusedTrajectoryPoint(mavros_msgs::PositionTarget &point);
  void publishWaypoints(bool hover);

  const ros::NodeHandle &nodeHandle() const { return nh_; }

 private:
  ros::NodeHandle nh_;

  nav_msgs::Path path_actual_;

  double avoid_radius_;
  geometry_msgs::Point avoid_centerpoint_;
  bool use_sphere_;

  // Subscribers
  ros::Subscriber pointcloud_sub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber velocity_sub_;
  ros::Subscriber state_sub_;
  ros::Subscriber clicked_point_sub_;
  ros::Subscriber clicked_goal_sub_;
  ros::Subscriber fcu_input_sub_;

  // Publishers
  ros::Publisher local_pointcloud_pub_;
  ros::Publisher ground_pointcloud_pub_;
  ros::Publisher front_pointcloud_pub_;
  ros::Publisher reprojected_points_pub_;
  ros::Publisher bounding_box_pub_;
  ros::Publisher groundbox_pub_;
  ros::Publisher height_map_pub_;
  ros::Publisher cached_pointcloud_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher path_pub_;
  ros::Publisher marker_rejected_pub_;
  ros::Publisher marker_blocked_pub_;
  ros::Publisher marker_candidates_pub_;
  ros::Publisher marker_selected_pub_;
  ros::Publisher marker_ground_pub_;
  ros::Publisher marker_FOV_pub_;
  ros::Publisher marker_goal_pub_;
  ros::Publisher ground_est_pub_;
  ros::Publisher avoid_sphere_pub_;
  ros::Publisher takeoff_pose_pub_;
  ros::Publisher offboard_pose_pub_;
  ros::Publisher initial_height_pub_;
  ros::Publisher complete_tree_pub_;
  ros::Publisher tree_path_pub_;
  ros::Publisher original_wp_pub_;
  ros::Publisher adapted_wp_pub_;
  ros::Publisher smoothed_wp_pub_;

  std::vector<float> algo_time;

  geometry_msgs::TwistStamped vel_msg_;
  bool armed_, offboard_, mission_, new_goal_;

  std::string depth_points_topic_;

  dynamic_reconfigure::Server<avoidance::LocalPlannerNodeConfig> server_;

  void dynamicReconfigureCallback(avoidance::LocalPlannerNodeConfig &config,
                                  uint32_t level);
  void positionCallback(const geometry_msgs::PoseStamped& msg);
  void pointCloudCallback(const sensor_msgs::PointCloud2& msg);
  void velocityCallback(const geometry_msgs::TwistStamped& msg);
  void stateCallback(const mavros_msgs::State& msg);
  void readParams();
  void publishPlannerData();
  void publishPath(const geometry_msgs::PoseStamped& msg);
  void initMarker(visualization_msgs::MarkerArray *marker,
                  nav_msgs::GridCells& path, const float red, const float green, const float blue);
  void publishMarkerBlocked(nav_msgs::GridCells& path_blocked);
  void publishMarkerRejected(nav_msgs::GridCells& path_rejected);
  void publishMarkerCandidates(nav_msgs::GridCells& path_candidates);
  void publishMarkerSelected(nav_msgs::GridCells& path_selected);
  void publishMarkerGround(nav_msgs::GridCells& path_ground);
  void publishMarkerFOV(nav_msgs::GridCells& FOV_cells);
  void clickedPointCallback(const geometry_msgs::PointStamped &msg);
  void clickedGoalCallback(const geometry_msgs::PoseStamped &msg);
  void fcuInputGoalCallback(const mavros_msgs::Trajectory &msg);

  void printPointInfo(double x, double y, double z);
  void publishGoal();
  void publishBox();
  void publishAvoidSphere();
  void publishGround();
  void publishReachHeight();
  void publishTree();
};

#endif  // LOCAL_PLANNER_LOCAL_PLANNER_NODE_H
