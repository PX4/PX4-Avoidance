#ifndef LOCAL_PLANNER_LOCAL_PLANNER_NODE_H
#define LOCAL_PLANNER_LOCAL_PLANNER_NODE_H

#include <iostream>
#include <math.h>
#include <string>
#include <thread>
#include <mutex>

#include <boost/bind.hpp>
#include <Eigen/Core>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <nav_msgs/GridCells.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>  // fromROSMsg
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h> // transformPointCloud
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include "std_msgs/String.h"
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "avoidance/common_ros.h"
#include "local_planner.h"

class LocalPlannerNode {

public:
  LocalPlannerNode();
  ~LocalPlannerNode();

  bool point_cloud_updated_;
  bool never_run_;
  bool position_received_;

  ros::Time pointcloud_time_now_;
  ros::Time pointcloud_time_old_;

  geometry_msgs::PoseStamped hover_point_;
  geometry_msgs::PoseStamped hover_current_pose_;

  LocalPlanner local_planner_;

  ros::Publisher log_name_pub_;
  ros::Publisher current_waypoint_pub_;
  ros::Publisher mavros_waypoint_pub_;
  ros::Publisher waypoint_pub_;
  ros::ServiceClient mavros_set_mode_client_;
  tf::TransformListener tf_listener_;

  std::timed_mutex variable_mutex_;

  void publishSetpoint(const geometry_msgs::PoseStamped wp, double mode);
  void threadFunction();

private:
  ros::NodeHandle nh_;

  nav_msgs::Path path_actual;

  int i = 0 ;

  // Subscribers
  ros::Subscriber pointcloud_sub_ ;
  ros::Subscriber pose_sub_ ;
  ros::Subscriber velocity_sub_ ;
  ros::Subscriber state_sub_ ;
  ros::Subscriber clicked_point_sub_;
  ros::Subscriber clicked_goal_sub_;

  // Publishers
  ros::Publisher local_pointcloud_pub_;
  ros::Publisher ground_pointcloud_pub_;
  ros::Publisher front_pointcloud_pub_;
  ros::Publisher reprojected_points_pub_;
  ros::Publisher bounding_box_pub_;
  ros::Publisher groundbox_pub_;
  ros::Publisher height_map_pub_;
  ros::Publisher cached_pointcloud_pub_ ;
  ros::Publisher marker_pub_;
  ros::Publisher path_pub_;
  ros::Publisher marker_rejected_pub_;
  ros::Publisher marker_blocked_pub_;
  ros::Publisher marker_candidates_pub_;
  ros::Publisher marker_selected_pub_;
  ros::Publisher marker_ground_pub_;
  ros::Publisher marker_goal_pub_;
  ros::Publisher ground_est_pub_;
  ros::Publisher avoid_sphere_pub_;
  ros::Publisher takeoff_pose_pub_;
  ros::Publisher initial_height_pub_;
  ros::Publisher complete_tree_pub_;

  std::vector<float> algo_time;

  std::string depth_points_topic_;

  dynamic_reconfigure::Server<avoidance::LocalPlannerNodeConfig> server_;

  void dynamicReconfigureCallback(avoidance::LocalPlannerNodeConfig & config, uint32_t level);
  void positionCallback(const geometry_msgs::PoseStamped msg);
  void pointCloudCallback(const sensor_msgs::PointCloud2 msg);
  void velocityCallback(const geometry_msgs::TwistStamped msg);
  void stateCallback(const mavros_msgs::State msg);
  void readParams();
  void publishAll();
  void publishPath(const geometry_msgs::PoseStamped msg);
  void initMarker(visualization_msgs::MarkerArray *marker, nav_msgs::GridCells path, float red, float green , float blue);
  void publishMarkerBlocked(nav_msgs::GridCells path_blocked);
  void publishMarkerRejected(nav_msgs::GridCells path_rejected);
  void publishMarkerCandidates(nav_msgs::GridCells path_candidates);
  void publishMarkerSelected(nav_msgs::GridCells path_selected);
  void publishMarkerGround(nav_msgs::GridCells path_ground);
  void clickedPointCallback(const geometry_msgs::PointStamped & msg);
  void clickedGoalCallback(const geometry_msgs::PoseStamped & msg);
  void printPointInfo(double x, double y, double z);
  void publishGoal();
  void publishBox();
  void publishAvoidSphere();
  void publishGround();
  void publishReachHeight();
  void publishTree();

};

#endif // LOCAL_PLANNER_LOCAL_PLANNER_NODE_H
