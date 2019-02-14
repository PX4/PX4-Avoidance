#ifndef LOCAL_PLANNER_FUNCTIONS_H
#define LOCAL_PLANNER_FUNCTIONS_H

#include "box.h"
#include "candidate_direction.h"
#include "common.h"
#include "cost_parameters.h"
#include "histogram.h"

#include <Eigen/Dense>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <nav_msgs/GridCells.h>
#include <nav_msgs/Path.h>

#include <queue>
#include <vector>

namespace avoidance {

/**
* @brief      crops the pointcloud so that only the points inside the bounding
*box around the vehicle position are considered
* @param[out] cropped_cloud, filtered pointcloud
* @param[out] closest_point, closest poincloud point to the vehicle position
* @param[out] distance_to_closest_point, 3D euclidean distance between the
*vehicle and closest_point [m]
* @param[out] counter_backoff, number of points closer than min_dist_backoff to
*the vehicle
* @param[in]  complete_cloud, array of pointclouds from the sensors
* @param[in]  min_cloud_size, minimum number of points in a pointcloud for it to
*be considered
* @param[in]  min_dist_backoff, distance bewteen the vehicle and a point in the
*cloud at which going backwards is considered [m]
* @param[in]  histogram_box, geometry definition of the bounding box
* @param[in]  position, current vehicle position
* @param[in]  min_realsense_dist, minimum sensor range [m]
**/
void filterPointCloud(
    pcl::PointCloud<pcl::PointXYZ>& cropped_cloud,
    Eigen::Vector3f& closest_point, float& distance_to_closest_point,
    int& counter_backoff,
    const std::vector<pcl::PointCloud<pcl::PointXYZ>>& complete_cloud,
    int min_cloud_size, float min_dist_backoff, Box histogram_box,
    const Eigen::Vector3f& position, float min_realsense_dist);

/**
* @brief      calculates the histogram cells within the Field of View
* @param[in]  h_FOV, horizontal Field of View [rad]
* @param[in]  v_FOV, vertical Field of View [rad]
* @param[out] z_FOV_idx, array of azimuth indexes inside the FOV
* @param[out] e_FOV_min, minimum elevation index inside the FOV
* @param[out] e_FOV_max, maximum elevation index inside the FOV
* @param[in]  yaw, vehicle yaw [rad]
* @param[in]  pitch, vehicle pitch [rad]
* @note azimuth angle is wrapped, elevation is not
**/
void calculateFOV(float h_FOV, float v_FOV, std::vector<int>& z_FOV_idx,
                  int& e_FOV_min, int& e_FOV_max, float yaw, float pitch);

/**
* @brief     calculates a histogram from older pointcloud data around the
*current vehicle postion
* @param[]   polar_histogram_est, histogram calculated from reprojected_points
* @param[in] reprojected_points, previous iterations occupied histogram bins
*reprojected in 3D space
* @param[in] reprojected_points_age, age of each reprojected point
* @param[]   position, current vehicle positon
**/
void propagateHistogram(
    Histogram& polar_histogram_est,
    const pcl::PointCloud<pcl::PointXYZ>& reprojected_points,
    const std::vector<int>& reprojected_points_age,
    const Eigen::Vector3f& position);

/**
* @brief      calculates a histogram from the current frame pointcloud around
*the current vehicle position
* @param[out] polar_histogram, represents cropped_cloud
* @param[in]  cropped_cloud, current frame filtered pointcloud
* @param[in]  position, current vehicle position
**/
void generateNewHistogram(Histogram& polar_histogram,
                          const pcl::PointCloud<pcl::PointXYZ>& cropped_cloud,
                          const Eigen::Vector3f& position);

/**
* @brief      merges together the histogram calculated with the current frame
*pointcloud with the one from previous frames
* @param[out] hist_empty, true if both the new and propagated histogram are
*empty
* @param      new_hist, histogram from current frame combined with
*propagated_hist
* @param[in]  propagated_hist, histofram calculated with points from previous
*frames
* @param[in]  waypoint_outside_FOV, true if the waypoint is outside the FOV
* @param[in]  z_FOV_idx, array of azimuth indexes inside the FOV
* @param[in]  e_FOV_min, minimum elevation index inside the FOV
* @param[in]  e_FOV_max, maximum elevation index inside the FOV
**/
void combinedHistogram(bool& hist_empty, Histogram& new_hist,
                       const Histogram& propagated_hist,
                       bool waypoint_outside_FOV,
                       const std::vector<int>& z_FOV_idx, int e_FOV_min,
                       int e_FOV_max);

/**
* @brief      compresses the histogram such that for each azimuth the minimum
*distance at the elevation inside the FOV is saved
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
* @param[in]  last_sent_waypoint, last position waypoint
* @param[in]  cost_params, weight for the cost function
* @param[in]  only_yawed, true if
* @param[out] cost_matrix,
**/
void getCostMatrix(const Histogram& histogram, const Eigen::Vector3f& goal,
                   const Eigen::Vector3f& position,
                   const Eigen::Vector3f& last_sent_waypoint,
                   costParameters cost_params, bool only_yawed,
                   Eigen::MatrixXf& cost_matrix);
/**
* @brief      classifies the candidate directions in increasing cost order
* @param[in]  matrix, cost matrix
* @param[in]  number_of_candidates, number of candiate direction to consider
* @param[out] candidate_vector, array of candidate polar direction arraged from
*the least to the most expensive
**/
void getBestCandidatesFromCostMatrix(
    const Eigen::MatrixXf& matrix, unsigned int number_of_candidates,
    std::vector<candidateDirection>& candidate_vector);

/**
* @brief   computes the cost of each direction in the polar histogram
* @param[] e_angle, elevation angle [deg]
* @param[] z_angle, azimuth angle [deg]
* @param[] goal, current goal position
* @param[] position, current vehicle position
* @param[] last_sent_waypoint, previous position waypoint
* @param[] cost_params, weights for goal oriented vs smooth behaviour
* @param[] only_yawed, true if the vehicle has only to yaw
* return   cost of direction (e_angle, z_angle)
**/
float costFunction(float e_angle, float z_angle, float obstacle_distance,
                   const Eigen::Vector3f& goal, const Eigen::Vector3f& position,
                   const Eigen::Vector3f& last_sent_waypoint,
                   costParameters cost_params, bool only_yawed);

/**
* @brief   max-median filtes the cost matrix
* @param   matrix, cost matrix
* @param[] smoothing_radius, median filter window size
**/
void smoothPolarMatrix(Eigen::MatrixXf& matrix, unsigned int smoothing_radius);

/**
* @brief       pads the cost matrix to wrap around elevation and azimuth when
*filtering
* @param[in]   matrix, cost matrix
* @param[in]   n_lines_padding, number of rows/columns to be added to matrix
* @param[out]  matrix_padded, cost matrix after padding
**/
void padPolarMatrix(const Eigen::MatrixXf& matrix, unsigned int n_lines_padding,
                    Eigen::MatrixXf& matrix_padded);

/**
* @brief   helper method to output on the console the histogram
* @param[] histogram, polar histogram
**/
void printHistogram(Histogram& histogram);

/**
* @brief      finds the minimum cost direction in the tree
* @param[out] p_pol, polar coordinates of the cheapest direction
* @param[in]  path_node_positions, array of expanded tree nodes
* @param[in]  position, current vehicle position
**/
bool getDirectionFromTree(
    PolarPoint& p_pol,
    const std::vector<geometry_msgs::Point>& path_node_positions,
    const Eigen::Vector3f& position);
}
#endif  // LOCAL_PLANNER_FUNCTIONS_H
