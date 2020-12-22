#pragma once

#include <ros/time.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <dynamic_reconfigure/server.h>
#include <safe_landing_planner/SLPGridMsg.h>
#include <safe_landing_planner/SafeLandingPlannerNodeConfig.h>

#include "grid.hpp"

namespace avoidance {

class SafeLandingPlanner {
 public:
  SafeLandingPlanner() = default;
  ~SafeLandingPlanner() = default;

  pcl::PointCloud<pcl::PointXYZ> cloud_;
  pcl::PointCloud<pcl::PointXYZI> visualization_cloud_;

  double timeout_critical_ = 0.5;
  double timeout_termination_ = 15.0;

  /**
  * @brief setter method for current vehicle position and orientation
  * @param[in] pos, vehicle position
  * @param[in] q, quaternion vehicle orientation
  **/
  void setPose(const Eigen::Vector3f& pos, const Eigen::Quaternionf& q);

  /**
  * @brif runs landing site detection algorithm
  **/
  void runSafeLandingPlanner();
  /**
  * @brief     sets parameters from ROS parameter server
  * @param     config, struct containing all the parameters
  * @param     level, bitmask to group together reconfigurable parameters
  **/
  void dynamicReconfigureSetParams(const safe_landing_planner::SafeLandingPlannerNodeConfig& config, uint32_t level);

  /**
  * @brief based on counter, standard devuation and mean, it decides if a cell
  *is landable
  **/
  void isLandingPossible();

  Eigen::Vector2i getPositionIndex() const { return pos_index_; };
  Grid getPreviousGrid() const { return previous_grid_; };
  Grid getGrid() const { return grid_; };
  int getSmoothingSize() const { return smoothing_size_; };

  safe_landing_planner::SLPGridMsg raw_grid_;

  bool play_rosbag_ = false;

 protected:
  Eigen::Vector3f position_ = Eigen::Vector3f::Zero();
  Eigen::Vector2i pos_index_ = Eigen::Vector2i(-1, -1);

  float n_points_thr_ = 1.f;
  float std_dev_thr_ = 0.1f;
  float grid_size_ = 10.f;
  float cell_size_ = 1.f;
  float mean_diff_thr_ = 0.3f;
  float alpha_ = 0.8f;
  int n_lines_padding_ = 1;
  int max_n_mean_diff_cells_ = 2;
  int grid_seq_ = 0;
  int smoothing_size_ = 1;
  int min_n_land_cells_ = 9;
  bool size_update_ = false;

  Grid grid_ = Grid(10.f, 1.f);
  Grid previous_grid_ = Grid(10.f, 1.f);

  /**
  * @brief process the pointcloud and calculate mean and variance for points in
  *the grid
  **/
  void processPointcloud();

  /**
  * @brief checks if a point cloud point is inside the 2D grid
  * @param[in] x, x coordinate of the pointcloud point
  * @param[in] y, y coordinate of the pointcloud point
  * @return true, if point is inside the grid
  **/
  bool isInsideGrid(float x, float y);
  /**
  * @brief computes the grid bin to which a point is mapped to
  * @param[in] x, x coordinate of the pointcloud point
  * @param[in] y, y coordinate of the pointcloud point
  * @returns indexes of the 2D grid
  **/
  Eigen::Vector2i computeGridIndexes(float x, float y);

  /**
  * @brief computes the online mean and variance of a grid bin on the z value
  * @param[in] prev_mean, previous bin mean value
  * @param[in] prev_variance, previous bin varinace value
  * @param[in] new_value, new data point to be averaged
  * @param[in] seq, number of data points already in the bin
  * @returns bin mean and variance
  **/
  std::pair<float, float> computeOnlineMeanVariance(float prev_mean, float prev_variance, float new_value, float seq);
  /**
  * @brief process the grid coming from a rosbag and map it to the datatypes
  * such that the algorithm can be run again
  **/
  void processRawGrid();
};
}
