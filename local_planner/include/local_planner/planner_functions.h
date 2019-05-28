#ifndef LOCAL_PLANNER_FUNCTIONS_H
#define LOCAL_PLANNER_FUNCTIONS_H

#include "avoidance/common.h"
#include "avoidance/histogram.h"
#include "box.h"
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
* @param[in]  histogram_box, geometry definition of the bounding box
* @param[in]  FOV, struct defining current field of view
* @param[in]  position, current vehicle position
* @param[in]  min_realsense_dist, minimum sensor range [m]
* @param[in]  max_age, maximum age (compute cycles) to keep data
* @param[in]  elapsed, time elapsed since last processing [s]
* @param[in]  min_num_points_per_cell, number of points from which on they will
*             be kept, less points are discarded as noise (careful: 0 is not
*             a valid input here)
**/
void processPointcloud(
    pcl::PointCloud<pcl::PointXYZI>& final_cloud,
    const std::vector<pcl::PointCloud<pcl::PointXYZ>>& complete_cloud,
    Box histogram_box, FOV& fov, const Eigen::Vector3f& position,
    float min_realsense_dist, int max_age, float elapsed_s,
    int min_num_points_per_cell);

/**
* @brief      calculates a histogram from the current frame pointcloud around
*             the current vehicle position
* @param[out] polar_histogram, represents cropped_cloud
* @param[in]  cropped_cloud, current frame filtered pointcloud
* @param[in]  position, current vehicle position
**/
void generateNewHistogram(Histogram& polar_histogram,
                          const pcl::PointCloud<pcl::PointXYZI>& cropped_cloud,
                          const Eigen::Vector3f& position);

/**
* @brief      compresses the histogram such that for each azimuth the minimum
*             distance at the elevation inside the FOV is saved
* @param[out] new_hist, compressed elevation histogram
* @param[int] input_hist, original histogram
**/
void compressHistogramElevation(Histogram& new_hist,
                                const Histogram& input_hist);
/**
* @brief      calculates each histogram bin cost and stores it in a cost matrix
* @param[in]  histogram, polar histogram representing obstacles
* @param[in]  goal, current goal position
* @param[in]  position, current vehicle position
* @param[in]  current vehicle heading in histogram angle convention [deg]
* @param[in]  last_sent_waypoint, last position waypoint
* @param[in]  cost_params, weight for the cost function
* @param[in]  only_yawed, true if
* @param[in]  parameter how far an obstacle is spread in the cost matrix
* @param[out] cost_matrix
* @param[out] image of the cost matrix for visualization
**/
void getCostMatrix(const Histogram& histogram, const Eigen::Vector3f& goal,
                   const Eigen::Vector3f& position,
                   const float yaw_angle_histogram_frame_deg,
                   const Eigen::Vector3f& last_sent_waypoint,
                   costParameters cost_params, bool only_yawed,
                   const float smoothing_margin_degrees,
                   Eigen::MatrixXf& cost_matrix,
                   std::vector<uint8_t>& image_data);

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
void generateCostImage(const Eigen::MatrixXf& cost_matrix,
                       const Eigen::MatrixXf& distance_matrix,
                       std::vector<uint8_t>& image_data);

/**
* @brief      classifies the candidate directions in increasing cost order
* @param[in]  matrix, cost matrix
* @param[in]  number_of_candidates, number of candidate direction to consider
* @param[out] candidate_vector, array of candidate polar direction arranged from
*             the least to the most expensive
**/
void getBestCandidatesFromCostMatrix(
    const Eigen::MatrixXf& matrix, unsigned int number_of_candidates,
    std::vector<candidateDirection>& candidate_vector);

/**
* @brief      computes the cost of each direction in the polar histogram
* @param[in]  e_angle, elevation angle [deg]
* @param[in]  z_angle, azimuth angle [deg]
* @param[in]  goal, current goal position
* @param[in]  position, current vehicle position
* @param[in]  position, current vehicle heading in histogram angle convention
*             [deg]
* @param[in]  last_sent_waypoint, previous position waypoint
* @param[in]  cost_params, weights for goal oriented vs smooth behaviour
* @param[out] distance_cost, cost component due to proximity to obstacles
* @param[out] other_costs, cost component due to goal and smoothness
**/
void costFunction(float e_angle, float z_angle, float obstacle_distance,
                  const Eigen::Vector3f& goal, const Eigen::Vector3f& position,
                  const float yaw_angle_histogram_frame_deg,
                  const Eigen::Vector3f& last_sent_waypoint,
                  costParameters cost_params, float& distance_cost,
                  float& other_costs);

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
void padPolarMatrix(const Eigen::MatrixXf& matrix, unsigned int n_lines_padding,
                    Eigen::MatrixXf& matrix_padded);

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
void printHistogram(Histogram& histogram);

/**
* @brief      finds the minimum cost direction in the tree
* @param[out] p_pol, polar coordinates of the cheapest direction
* @param[in]  path_node_positions, array of expanded tree nodes
* @param[in]  position, current vehicle position
**/
bool getDirectionFromTree(
    PolarPoint& p_pol, const std::vector<Eigen::Vector3f>& path_node_positions,
    const Eigen::Vector3f& position, const Eigen::Vector3f& goal);
}
#endif  // LOCAL_PLANNER_FUNCTIONS_H
