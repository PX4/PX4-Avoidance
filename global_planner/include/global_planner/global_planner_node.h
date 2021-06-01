#ifndef GLOBAL_PLANNER_GLOBAL_PLANNER_NODE_H
#define GLOBAL_PLANNER_GLOBAL_PLANNER_NODE_H

#include <math.h>
#include <stdio.h>
#include <boost/bind.hpp>
#include <mutex>
#include <set>
#include <string>
#include "rclcpp/rclcpp.hpp"

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_trajectory_waypoint.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.h>

#include "avoidance/avoidance_node.h"
#include "global_planner/global_planner.h"

#ifndef DISABLE_SIMULATION
#include <avoidance/rviz_world_loader.h>
#endif

namespace global_planner {

using std::placeholders::_1;
using namespace std::chrono_literals;

struct cameraData {
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
};

class GlobalPlannerNode : public rclcpp::Node {
 public:
  // TODO: Deque instead of vector
  GlobalPlanner global_planner_;
  std::vector<GoalCell> waypoints_;  // Intermediate goals, from file, mavros
                                     // mission or intermediate goals
  GlobalPlannerNode();
  ~GlobalPlannerNode();

 private:
  std::mutex mutex_;

  // Subscribers
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_full_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr global_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr move_base_simple_sub_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_temp_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr smooth_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr actual_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr explored_cells_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr global_goal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr global_temp_goal_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_waypoint_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;

  rclcpp::Time start_time_;
  rclcpp::Time last_wp_time_;

  rclcpp::TimerBase::SharedPtr gp_cmdloop_timer_;
  rclcpp::TimerBase::SharedPtr gp_plannerloop_timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

  nav_msgs::msg::Path actual_path_;
  geometry_msgs::msg::Point start_pos_;
  geometry_msgs::msg::PoseStamped current_goal_;
  geometry_msgs::msg::PoseStamped last_goal_;
  geometry_msgs::msg::PoseStamped last_pos_;
  px4_msgs::msg::VehicleStatus last_vehicle_status_;
  sensor_msgs::msg::PointCloud2 pointcloud2_;

  std::vector<geometry_msgs::msg::PoseStamped> last_clicked_points;
  std::vector<geometry_msgs::msg::PoseStamped> path_;
  std::vector<cameraData> cameras_;

  int num_octomap_msg_ = 0;
  int num_local_pos_msg_ = 0;
  int num_global_pos_msg_ = 0;
  std::chrono::milliseconds gp_cmdloop_dt_;
  std::chrono::milliseconds gp_plannerloop_dt_;
  double mapupdate_dt_;
  double min_speed_;
  double speed_ = min_speed_;
  double start_yaw_;
  bool position_received_;
  std::string frame_id_;
  std::string camera_frame_id_;
  
  double clicked_goal_alt_;
  double clicked_goal_radius_;
  bool hover_;
  int simplify_iterations_;
  double simplify_margin_;

  avoidance::AvoidanceNode avoidance_node_;
#ifndef DISABLE_SIMULATION
  avoidance::WorldVisualizer::SharedPtr world_visualizer_;
  rclcpp::executors::MultiThreadedExecutor world_visualizer_executor_;
  rclcpp::TimerBase::SharedPtr world_visualizer_timer_;
#endif
  void readParams();
  void initializeCameraSubscribers(std::vector<std::string>& camera_topics);
  void setNewGoal(const GoalCell& goal);
  void popNextGoal();
  void planPath();
  void setIntermediateGoal();
  bool isCloseToGoal();
  void setCurrentPath(const std::vector<geometry_msgs::msg::PoseStamped>& poses);
  void localPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  void globalPositionCallback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);
  void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void clickedPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void moveBaseSimpleCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void octomapFullCallback(const octomap_msgs::msg::Octomap::SharedPtr msg);
  void depthCameraCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void attitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
  void cmdLoopCallback();
  void plannerLoopCallback();
  void publishGoal(const GoalCell& goal);
  void publishPath();
  void publishSetpoint();
  void printPointInfo(double x, double y, double z);
};

}  // namespace global_planner

#endif  // GLOBAL_PLANNER_GLOBAL_PLANNER_NODE_H