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

void filterPointCloud(
    pcl::PointCloud<pcl::PointXYZ>& cropped_cloud,
    Eigen::Vector3f& closest_point, double& distance_to_closest_point,
    int& counter_backoff,
    const std::vector<pcl::PointCloud<pcl::PointXYZ>>& complete_cloud,
    double min_cloud_size, double min_dist_backoff, Box histogram_box,
    const Eigen::Vector3f& position, double min_realsense_dist);
void calculateFOV(double h_FOV, double v_FOV, std::vector<int>& z_FOV_idx,
                  int& e_FOV_min, int& e_FOV_max, double yaw, double pitch);
void propagateHistogram(
    Histogram& polar_histogram_est,
    const pcl::PointCloud<pcl::PointXYZ>& reprojected_points,
    const std::vector<int>& reprojected_points_age,
    const geometry_msgs::PoseStamped& position);
void generateNewHistogram(Histogram& polar_histogram,
                          const pcl::PointCloud<pcl::PointXYZ>& cropped_cloud,
                          const Eigen::Vector3f& position);
void combinedHistogram(bool& hist_empty, Histogram& new_hist,
                       const Histogram& propagated_hist,
                       bool waypoint_outside_FOV,
                       const std::vector<int>& z_FOV_idx, int e_FOV_min,
                       int e_FOV_max);
void compressHistogramElevation(Histogram& new_hist,
                                const Histogram& input_hist);
void getCostMatrix(const Histogram& histogram, const Eigen::Vector3f& goal,
                   const Eigen::Vector3f& position,
                   const Eigen::Vector3f& last_sent_waypoint,
                   costParameters cost_params, bool only_yawed,
                   Eigen::MatrixXd& cost_matrix);
void getBestCandidatesFromCostMatrix(
    const Eigen::MatrixXd& matrix, unsigned int number_of_candidates,
    std::vector<candidateDirection>& candidate_vector);
double costFunction(double e_angle, double z_angle, float obstacle_distance,
                    const Eigen::Vector3f& goal,
                    const Eigen::Vector3f& position,
                    const Eigen::Vector3f& last_sent_waypoint,
                    costParameters cost_params, bool only_yawed);
void smoothPolarMatrix(Eigen::MatrixXd& matrix, unsigned int smoothing_radius);
void padPolarMatrix(const Eigen::MatrixXd& matrix, unsigned int n_lines_padding,
                    Eigen::MatrixXd& matrix_padded);
void printHistogram(Histogram& histogram);
bool getDirectionFromTree(
    PolarPoint& p_pol,
    const std::vector<geometry_msgs::Point>& path_node_positions,
    const Eigen::Vector3f& position);
}
#endif  // LOCAL_PLANNER_FUNCTIONS_H
