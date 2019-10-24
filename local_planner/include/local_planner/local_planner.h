#ifndef LOCAL_PLANNER_LOCAL_PLANNER_H
#define LOCAL_PLANNER_LOCAL_PLANNER_H

#include <sensor_msgs/image_encodings.h>
#include "avoidance/histogram.h"
#include "avoidance_output.h"
#include "candidate_direction.h"
#include "cost_parameters.h"
#include "planner_functions.h"

#include <dynamic_reconfigure/server.h>
#include <local_planner/LocalPlannerNodeConfig.h>

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <nav_msgs/GridCells.h>
#include <nav_msgs/Path.h>

#include <ros/time.h>
#include <deque>
#include <string>
#include <vector>

namespace avoidance {

class StarPlanner;
class TreeNode;

class LocalPlanner {
 private:
  int children_per_node_;
  int n_expanded_nodes_;
  int min_num_points_per_cell_ = 3;

  float min_sensor_range_ = 0.2f;
  float max_sensor_range_ = 12.0f;
  float smoothing_margin_degrees_ = 30.f;
  float max_point_age_s_ = 10;
  float yaw_fcu_frame_deg_ = 0.0f;
  float pitch_fcu_frame_deg_ = 0.0f;

  std::vector<FOV> fov_fcu_frame_;

  ros::Time last_path_time_;
  ros::Time last_pointcloud_process_time_;

  std::vector<int> closed_set_;
  std::vector<TreeNode> tree_;
  std::unique_ptr<StarPlanner> star_planner_;
  costParameters cost_params_;

  pcl::PointCloud<pcl::PointXYZI> final_cloud_;

  Eigen::Vector3f position_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f velocity_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f goal_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f prev_goal_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f closest_pt_ = Eigen::Vector3f::Zero();

  Histogram polar_histogram_ = Histogram(ALPHA_RES);
  Histogram to_fcu_histogram_ = Histogram(ALPHA_RES);
  Eigen::MatrixXf cost_matrix_;

  /**
  * @brief     fills message to send histogram to the FCU
  **/
  void updateObstacleDistanceMsg(Histogram hist);
  /**
  * @brief     fills message to send empty histogram to the FCU
  **/
  void updateObstacleDistanceMsg();
  /**
  * @brief     creates a polar histogram representation of the pointcloud
  * @param[in] send_to_fcu, true if the histogram is sent to the FCU
  **/
  void create2DObstacleRepresentation(bool send_to_fcu);
  /**
  * @brief     generates an image represention of the polar histogram
  * @param     histogram, polar histogram representing obstacles
  * @returns   histogram image
  **/
  void generateHistogramImage(Histogram& histogram);

 public:
  std::vector<uint8_t> histogram_image_data_;
  std::vector<uint8_t> cost_image_data_;
  bool currently_armed_ = false;

  double timeout_startup_ = 20.0;
  double timeout_critical_ = 0.5;
  double timeout_termination_ = 20.0;
  float speed_ = 1.0f;
  float mission_item_speed_ = NAN;

  ModelParameters px4_;  // PX4 Firmware paramters

  sensor_msgs::LaserScan distance_data_ = {};
  Eigen::Vector3f last_sent_waypoint_ = Eigen::Vector3f::Zero();

  // original_cloud_vector_ contains n complete clouds from the cameras
  std::vector<pcl::PointCloud<pcl::PointXYZ>> original_cloud_vector_;

  LocalPlanner();
  ~LocalPlanner();

  /**
  * @brief     setter method for vehicle position, orientation and velocity
  * @param[in] pos, vehicle position coming from the FCU
  * @param[in] vel, vehicle velocity in the FCU frame
  * @param[in] q, vehicle orientation message coming from the FCU
  **/
  void setState(const Eigen::Vector3f& pos, const Eigen::Vector3f& vel, const Eigen::Quaternionf& q);

  /**
  * @brief     setter method for mission goal
  * @param[in] goal, goal message coming from the FCU
  **/
  void setGoal(const Eigen::Vector3f& goal);

  /**
  * @brief     setter method for previous mission goal
  * @param[in] prev_goal, previous goal message coming from the FCU
  **/
  void setPreviousGoal(const Eigen::Vector3f& prev_goal);

  /**
  * @brief     setter method for field of view
  * @param[in] index of the camera
  * @param[in] field of view structure of the camera
  */
  void setFOV(int i, const FOV& fov);

  /**
  * @brief     Getters for the FOV
  */
  float getHFOV(int i) { return i < fov_fcu_frame_.size() ? fov_fcu_frame_[i].h_fov_deg : 0.f; }
  float getVFOV(int i) { return i < fov_fcu_frame_.size() ? fov_fcu_frame_[i].v_fov_deg : 0.f; }
  const std::vector<FOV>& getFOV() const { return fov_fcu_frame_; }
  float getSensorRange() const { return max_sensor_range_; }

  /**
  * @brief     getter method for current goal
  * @returns   position of the goal
  **/
  Eigen::Vector3f getGoal() const;
  /**
  * @brief    setter method for mission goal
  **/
  void applyGoal();
  /**
  * @brief     sets parameters from ROS parameter server
  * @param     config, struct containing all the parameters
  * @param     level, bitmask to group together reconfigurable parameters
  **/
  void dynamicReconfigureSetParams(avoidance::LocalPlannerNodeConfig& config, uint32_t level);

  /**
  * @brief     getter method for current vehicle orientation
  * @returns   vehicle orientation in the fcu convention of the world frame
  **/
  float getOrientation() const { return yaw_fcu_frame_deg_; }

  /**
  * @brief     getter method for current vehicle position
  * @returns   vehicle position
  **/
  Eigen::Vector3f getPosition() const;

  /**
  * @brief     getter method to visualize the pointcloud used for planning
  * @returns   reference to pointcloud
  **/
  const pcl::PointCloud<pcl::PointXYZI>& getPointcloud() const;

  /**
  * @brief     getter method to visualize the tree in rviz
  * @param[in] tree, the whole tree built during planning (vector of nodes)
  * @param[in] closed_set, velocity message coming from the FCU
  * @param[in] path_node_positions, velocity message coming from the FCU
  **/
  void getTree(std::vector<TreeNode>& tree, std::vector<int>& closed_set,
               std::vector<Eigen::Vector3f>& path_node_positions) const;
  /**
  * @brief     getter method for obstacle distance information
  * @param     obstacle_distance, obstacle distance message to fill
  **/
  void getObstacleDistanceData(sensor_msgs::LaserScan& obstacle_distance);

  /**
  * @brief     getter method of the local planner algorithm
  * @param[in] output of a local planner iteration
  **/
  avoidanceOutput getAvoidanceOutput() const;

  /**
  * @brief     determines the way the obstacle is avoided and the algorithm to
  *            use
  **/
  void determineStrategy();
  /**
  * @brief     starts a iteration of the local planner algorithm
  **/
  void runPlanner();

  /**
  * @brief     setter method for PX4 Firmware paramters
  **/
  void setDefaultPx4Parameters();
};
}

#endif  // LOCAL_PLANNER_H
