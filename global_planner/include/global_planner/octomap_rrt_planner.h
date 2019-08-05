//  June/2019, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#ifndef OCTOMAP_RRT_PLANNER_H
#define OCTOMAP_RRT_PLANNER_H

#include <stdio.h>
#include <cstdlib>
#include <mutex>
#include <sstream>
#include <string>

#include <Eigen/Dense>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>

#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <mavros_msgs/Trajectory.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <avoidance/avoidance_node.h>
#include <avoidance/common.h>
#include <global_planner/octomap_ompl_rrt.h>

#ifndef DISABLE_SIMULATION
#include <avoidance/rviz_world_loader.h>
#endif

using namespace std;
using namespace Eigen;
namespace ob = ompl::base;
namespace og = ompl::geometric;

struct cameraData {
  ros::Subscriber pointcloud_sub_;
};

class OctomapRrtPlanner {
 private:
  std::mutex mutex_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher trajectory_pub_;
  ros::Publisher global_path_pub_;
  ros::Publisher pointcloud_pub_;

  ros::Subscriber pose_sub_;
  ros::Subscriber octomap_full_sub_;
  ros::Subscriber move_base_simple_sub_;
  ros::Subscriber desiredtrajectory_sub_;

  ros::Timer cmdloop_timer_;
  ros::Timer plannerloop_timer_;
  ros::CallbackQueue cmdloop_queue_;
  ros::CallbackQueue plannerloop_queue_;
  std::unique_ptr<ros::AsyncSpinner> cmdloop_spinner_;
  std::unique_ptr<ros::AsyncSpinner> plannerloop_spinner_;

  ros::Time last_wp_time_;
  ros::Time start_time_;

  double cmdloop_dt_;
  double plannerloop_dt_;
  bool hover_;
  int num_octomap_msg_;
  std::string frame_id_;

  Eigen::Vector3d local_position_, local_velocity_;
  Eigen::Vector3d reference_pos_;
  Eigen::Quaternionf reference_att_;
  Eigen::Vector3d goal_;

  std::vector<Eigen::Vector3d> current_path_;
  std::vector<cameraData> cameras_;
  tf::TransformListener listener_;

  octomap::OcTree* octree_ = NULL;

  OctomapOmplRrt rrt_planner_;
  avoidance::AvoidanceNode avoidance_node_;
#ifndef DISABLE_SIMULATION
  std::unique_ptr<avoidance::WorldVisualizer> world_visualizer_;
#endif
  void cmdLoopCallback(const ros::TimerEvent& event);
  void plannerLoopCallback(const ros::TimerEvent& event);
  void octomapFullCallback(const octomap_msgs::Octomap& msg);
  void positionCallback(const geometry_msgs::PoseStamped& msg);
  void velocityCallback(const geometry_msgs::TwistStamped& msg);
  void DesiredTrajectoryCallback(const mavros_msgs::Trajectory& msg);
  void moveBaseSimpleCallback(const geometry_msgs::PoseStamped& msg);
  void depthCameraCallback(const sensor_msgs::PointCloud2& msg);
  void publishSetpoint();
  void initializeCameraSubscribers(std::vector<std::string>& camera_topics);
  void publishPath();
  geometry_msgs::PoseStamped vector3d2PoseStampedMsg(Eigen::Vector3d position);
  void updateReference(ros::Time current_time);

 public:
  OctomapRrtPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  virtual ~OctomapRrtPlanner();
  void planWithSimpleSetup();
};

#endif
