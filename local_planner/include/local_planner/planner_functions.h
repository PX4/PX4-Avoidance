#ifndef LOCAL_PLANNER_FUNCTIONS_H
#define LOCAL_PLANNER_FUNCTIONS_H

#include "avoidance/common.h"
#include "avoidance/histogram.h"
#include "candidate_direction.h"
#include "cost_parameters.h"

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <queue>
#include <vector>

namespace avoidance {

/**
* @brief      crops and subsamples the incomming data, then combines it with
*             the data from the last timestep
* @param      final_cloud, processed data to be used for planning
* @param[in]  complete_cloud, array of pointclouds from the sensors
* @param[in]  FOV, struct defining current field of view
* @param[in]  position, current vehicle position
* @param[in]  min_realsense_dist, minimum sensor range [m]
* @param[in]  max_age, maximum age in seconds to keep data
* @param[in]  elapsed, time elapsed since last processing [s]
* @param[in]  min_num_points_per_cell, number of points from which on they will
*             be kept, less points are discarded as noise (careful: 0 is not
*             a valid input here)
**/
void processPointcloud(pcl::PointCloud<pcl::PointXYZI>& final_cloud,
                       const std::vector<pcl::PointCloud<pcl::PointXYZ>>& complete_cloud, const std::vector<FOV>& fov,
                       float yaw_fcu_frame_deg, float pitch_fcu_frame_deg, const Eigen::Vector3f& position,
                       float min_sensor_range, float max_sensor_range, float max_age, float elapsed_s,
                       int min_num_points_per_cell);

/**
* @brief      calculates a histogram from the current frame pointcloud around
*             the current vehicle position
* @param[out] polar_histogram, represents cropped_cloud
* @param[in]  cropped_cloud, current frame filtered pointcloud
* @param[in]  position, current vehicle position
**/
void generateNewHistogram(Histogram& polar_histogram, const pcl::PointCloud<pcl::PointXYZI>& cropped_cloud,
                          const Eigen::Vector3f& position);

/**
* @brief      compresses the histogram such that for each azimuth the minimum
*             distance at the elevation inside the FOV is saved
* @param[out] new_hist, compressed elevation histogram
* @param[in] input_hist, original histogram
**/
void compressHistogramElevation(Histogram& new_hist, const Histogram& input_hist, const Eigen::Vector3f& position);

/**
* @brief      calculates each histogram bin cost and stores it in a cost matrix
* @param[in]  histogram, polar histogram representing obstacles
* @param[in]  goal, current goal position
* @param[in]  position, current vehicle position
* @param[in]  current vehicle heading in histogram angle convention [deg]
* @param[in]  last_sent_waypoint, last position waypoint
* @param[in]  cost_params, weight for the cost function
* @param[in]  parameter how far an obstacle is spread in the cost matrix
* @param[in]  closest_pt, vehicle position projection on the line previous-current goal
* @param[in]  max_sensor_range, maximum distance at which the sensor detects objects
* @param[in]  min_sensor_range, minimum distance at which the sensor detects objects
* @param[out] cost_matrix
* @param[out] image of the cost matrix for visualization
**/
void getCostMatrix(const Histogram& histogram, const Eigen::Vector3f& goal, const Eigen::Vector3f& position,
                   const Eigen::Vector3f& velocity, const costParameters& cost_params, float smoothing_margin_degrees,
                   const Eigen::Vector3f& closest_pt, const float max_sensor_range, const float min_sensor_range,
                   Eigen::MatrixXf& cost_matrix, std::vector<uint8_t>& image_data);

/**
* @brief      get the index in the data vector of a color image
*             from the histogram index
* @param[in]  histogram index e,z and color (0=red, 1=green, 2=blue)
* @param[out] index in image data vector
**/
int colorImageIndex(int e_ind, int z_ind, int color);

/**
* @brief      transform cost_matrix into an image
* @param[in]  cost matrices
* @param[out] image for cost matrix visualization
**/
void generateCostImage(const Eigen::MatrixXf& cost_matrix, const Eigen::MatrixXf& distance_matrix,
                       std::vector<uint8_t>& image_data);

/**
* @brief      classifies the candidate directions in increasing cost order
* @param[in]  matrix, cost matrix
* @param[in]  number_of_candidates, number of candidate direction to consider
* @param[out] candidate_vector, array of candidate polar direction arranged from
*             the least to the most expensive
**/
void getBestCandidatesFromCostMatrix(const Eigen::MatrixXf& matrix, unsigned int number_of_candidates,
                                     std::vector<candidateDirection>& candidate_vector);

/**
* @brief      computes the cost of each direction in the polar histogram
* @param[in]  PolarPoint of the candidate direction
* @param[in]  goal, current goal position
* @param[in]  position, current vehicle position
* @param[in]  velocity, current vehicle velocity
* @param[in]  cost_params, weights for goal oriented vs smooth behaviour
* @param[in]  closest_pt, vehicle position projection on the line previous-current goal
* @param[in]  is_obstacle_facing_goal, true if there is an obstacle in the goal direction
* @returns    a pair with the first value representing the distance cost, and the second the sum of all other costs
**/
std::pair<float, float> costFunction(const PolarPoint& candidate_polar, float obstacle_distance,
                                     const Eigen::Vector3f& goal, const Eigen::Vector3f& position,
                                     const Eigen::Vector3f& velocity, const costParameters& cost_params,
                                     const Eigen::Vector3f& closest_pt, const bool is_obstacle_facing_goal);

/**
* @brief      max-median filtes the cost matrix
* @param      matrix, cost matrix
* @param[in]  smoothing_radius, median filter window size
**/
void smoothPolarMatrix(Eigen::MatrixXf& matrix, unsigned int smoothing_radius);

/**
* @brief      pads the cost matrix to wrap around elevation and azimuth when
*             filtering
* @param[in]  matrix, cost matrix
* @param[in]  n_lines_padding, number of rows/columns to be added to matrix
* @param[out] matrix_padded, cost matrix after padding
**/
void padPolarMatrix(const Eigen::MatrixXf& matrix, unsigned int n_lines_padding, Eigen::MatrixXf& matrix_padded);

/**
 * @brief     creates an 1d array with size 2*radius + 1 in length and fills it
 *            with a conic kernel value
 * @param[in] radius the radius of the kernel
 * @return    the smoothing kernel
 **/
Eigen::ArrayXf getConicKernel(int radius);

/**
* @brief      helper method to output on the console the histogram
* @param[in]  histogram, polar histogram
**/
void printHistogram(const Histogram& histogram);

/**
* @brief      Returns a setpoint that lies on the given path
* @param[in]  vector of nodes defining the path, with the last node of the path at index 0
* @param[in]  ros time of path generation
* @param[in]  velocity, scalar value for the norm of the current vehicle velocity
* @param[out] setpoint on the tree toward which the drone should fly
* @returns    boolean indicating whether the tree was valid
**/
bool getSetpointFromPath(const std::vector<Eigen::Vector3f>& path, const ros::Time& path_generation_time,
                         float velocity, const ros::Time& current_time, Eigen::Vector3f& setpoint);
}
#endif  // LOCAL_PLANNER_FUNCTIONS_H
