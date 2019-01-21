#ifndef LOCAL_PLANNER_LOCAL_PLANNER_NODE_H
#define LOCAL_PLANNER_LOCAL_PLANNER_NODE_H

#include "avoidance/common_ros.h"
#include "avoidance_output.h"
#include "rviz_world_loader.h"

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <mavros_msgs/Altitude.h>
#include <mavros_msgs/CompanionProcessStatus.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Trajectory.h>
#include <nav_msgs/GridCells.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>  // fromROSMsg
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>  // transformPointCloud
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Core>
#include <boost/bind.hpp>

#include <dynamic_reconfigure/server.h>
#include <local_planner/LocalPlannerNodeConfig.h>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

namespace avoidance {

class LocalPlanner;
class WaypointGenerator;

struct cameraData {
  std::string topic_;
  ros::Subscriber pointcloud_sub_;
  ros::Subscriber camera_info_sub_;
  sensor_msgs::PointCloud2 newest_cloud_msg_;
  bool received_;
};

enum class MAV_STATE {
  MAV_STATE_UNINIT,
  MAV_STATE_BOOT,
  MAV_STATE_CALIBRATIN,
  MAV_STATE_STANDBY,
  MAV_STATE_ACTIVE,
  MAV_STATE_CRITICAL,
  MAV_STATE_EMERGENCY,
  MAV_STATE_POWEROFF,
  MAV_STATE_FLIGHT_TERMINATION,
};

class LocalPlannerNode {
 public:
  LocalPlannerNode();
  ~LocalPlannerNode();

  mavros_msgs::CompanionProcessStatus status_msg_;

  std::string world_path_;
  bool never_run_ = true;
  bool position_received_ = false;
  bool disable_rise_to_goal_altitude_;
  bool accept_goal_input_topic_;

  std::atomic<bool> should_exit_{false};

  double curr_yaw_;

  std::vector<cameraData> cameras_;

  ros::CallbackQueue pointcloud_queue_;
  ros::CallbackQueue main_queue_;

  std::vector<geometry_msgs::Point> path_node_positions_;
  geometry_msgs::PoseStamped hover_point_;
  geometry_msgs::PoseStamped newest_pose_;
  geometry_msgs::PoseStamped last_pose_;
  geometry_msgs::Point newest_waypoint_position_;
  geometry_msgs::Point last_waypoint_position_;
  geometry_msgs::PoseStamped goal_msg_;

  const ros::Duration pointcloud_timeout_hover_ = ros::Duration(0.4);
  const ros::Duration pointcloud_timeout_land_ = ros::Duration(10);

  ros::Time last_wp_time_;
  ros::Time t_status_sent_;

  std::unique_ptr<LocalPlanner> local_planner_;
  std::unique_ptr<WaypointGenerator> wp_generator_;

  ros::Publisher world_pub_;
  ros::Publisher drone_pub_;
  ros::Publisher current_waypoint_pub_;
  ros::Publisher mavros_pos_setpoint_pub_;
  ros::Publisher mavros_vel_setpoint_pub_;
  ros::Publisher mavros_obstacle_free_path_pub_;
  ros::Publisher mavros_obstacle_distance_pub_;
  ros::Publisher waypoint_pub_;
  ros::ServiceClient mavros_set_mode_client_;
  ros::Publisher mavros_system_status_pub_;
  tf::TransformListener tf_listener_;

  std::mutex running_mutex_;  ///< guard against concurrent access to input &
                              /// output data (point cloud, position, ...)

  std::mutex data_ready_mutex_;
  bool data_ready_ = false;
  std::condition_variable data_ready_cv_;

  void publishSetpoint(const geometry_msgs::Twist& wp,
                       waypoint_choice& waypoint_type);
  void threadFunction();
  void getInterimWaypoint(geometry_msgs::PoseStamped& wp,
                          geometry_msgs::Twist& wp_vel);
  bool canUpdatePlannerInfo();
  void updatePlannerInfo();
  size_t numReceivedClouds();
  void transformPoseToTrajectory(mavros_msgs::Trajectory& obst_avoid,
                                 geometry_msgs::PoseStamped pose);
  void transformVelocityToTrajectory(mavros_msgs::Trajectory& obst_avoid,
                                     geometry_msgs::Twist vel);
  void fillUnusedTrajectoryPoint(mavros_msgs::PositionTarget& point);
  void publishWaypoints(bool hover);

  const ros::NodeHandle& nodeHandle() const { return nh_; }

 private:
  ros::NodeHandle nh_;
  avoidance::LocalPlannerNodeConfig rqt_param_config_;

  mavros_msgs::Altitude ground_distance_msg_;
  int path_length_ = 0;

  // Subscribers
  ros::Subscriber pose_sub_;
  ros::Subscriber velocity_sub_;
  ros::Subscriber state_sub_;
  ros::Subscriber clicked_point_sub_;
  ros::Subscriber clicked_goal_sub_;
  ros::Subscriber fcu_input_sub_;
  ros::Subscriber goal_topic_sub_;
  ros::Subscriber distance_sensor_sub_;

  // Publishers
  ros::Publisher local_pointcloud_pub_;
  ros::Publisher front_pointcloud_pub_;
  ros::Publisher reprojected_points_pub_;
  ros::Publisher bounding_box_pub_;
  ros::Publisher ground_measurement_pub_;
  ros::Publisher height_map_pub_;
  ros::Publisher cached_pointcloud_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher path_actual_pub_;
  ros::Publisher path_waypoint_pub_;
  ros::Publisher marker_goal_pub_;
  ros::Publisher takeoff_pose_pub_;
  ros::Publisher offboard_pose_pub_;
  ros::Publisher initial_height_pub_;
  ros::Publisher complete_tree_pub_;
  ros::Publisher tree_path_pub_;
  ros::Publisher original_wp_pub_;
  ros::Publisher adapted_wp_pub_;
  ros::Publisher smoothed_wp_pub_;
  ros::Publisher histogram_image_pub_;

  std::vector<float> algo_time;

  geometry_msgs::TwistStamped vel_msg_;
  bool armed_, offboard_, mission_, new_goal_;

  dynamic_reconfigure::Server<avoidance::LocalPlannerNodeConfig>* server_;
  boost::recursive_mutex config_mutex_;

  void dynamicReconfigureCallback(avoidance::LocalPlannerNodeConfig& config,
                                  uint32_t level);
  void initializeCameraSubscribers(std::vector<std::string>& camera_topics);
  void positionCallback(const geometry_msgs::PoseStamped& msg);
  void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg,
                          int index);
  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg,
                          int index);
  void velocityCallback(const geometry_msgs::TwistStamped& msg);
  void stateCallback(const mavros_msgs::State& msg);
  void readParams();
  void publishPlannerData();
  void publishPaths();
  void clickedPointCallback(const geometry_msgs::PointStamped& msg);
  void clickedGoalCallback(const geometry_msgs::PoseStamped& msg);
  void updateGoalCallback(const visualization_msgs::MarkerArray& msg);
  void fcuInputGoalCallback(const mavros_msgs::Trajectory& msg);
  void distanceSensorCallback(const mavros_msgs::Altitude& msg);

  void printPointInfo(double x, double y, double z);
  void publishGoal();
  void publishBox();
  void publishReachHeight();
  void publishTree();
  void publishHistogramImage();
  void publishGround();
};
}
#endif  // LOCAL_PLANNER_LOCAL_PLANNER_NODE_H
