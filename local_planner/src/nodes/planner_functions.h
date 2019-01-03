#ifndef LOCAL_PLANNER_FUNCTIONS_H
#define LOCAL_PLANNER_FUNCTIONS_H

#include "box.h"
#include "histogram.h"

#include <Eigen/Dense>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <nav_msgs/GridCells.h>
#include <nav_msgs/Path.h>

#include <vector>

namespace avoidance {

void initGridCells(nav_msgs::GridCells& cell);
double adaptSafetyMarginHistogram(double dist_to_closest_point,
                                  double cloud_size, double min_cloud_size);
void filterPointCloud(
    pcl::PointCloud<pcl::PointXYZ>& cropped_cloud,
    Eigen::Vector3f& closest_point, double& distance_to_closest_point,
    int& counter_backoff,
    const std::vector<pcl::PointCloud<pcl::PointXYZ>>& complete_cloud,
    double min_cloud_size, double min_dist_backoff, Box histogram_box,
    const Eigen::Vector3f& position, double min_realsense_dist);
void calculateFOV(double h_FOV, double v_FOV, std::vector<int>& z_FOV_idx,
                  int& e_FOV_min, int& e_FOV_max, double yaw, double pitch);
void propagateHistogram(Histogram& polar_histogram_est,
                        const pcl::PointCloud<pcl::PointXYZ>& reprojected_points,
                        const std::vector<double>& reprojected_points_age,
                        const std::vector<double>& reprojected_points_dist,
                        geometry_msgs::PoseStamped position);
void generateNewHistogram(Histogram& polar_histogram,
                          const pcl::PointCloud<pcl::PointXYZ>& cropped_cloud,
                          geometry_msgs::PoseStamped position);
void combinedHistogram(bool& hist_empty, Histogram& new_hist,
                       const Histogram& propagated_hist,
                       bool waypoint_outside_FOV,
                       const std::vector<int>& z_FOV_idx, int e_FOV_min,
                       int e_FOV_max);
void compressHistogramElevation(Histogram& new_hist,
                                const Histogram& input_hist);
double costFunction(int e, int z, const nav_msgs::GridCells& path_waypoints,
                    const Eigen::Vector3f& goal,
                    const Eigen::Vector3f& position,
                    const Eigen::Vector3f& position_old, double goal_cost_param,
                    double smooth_cost_param,
                    double height_change_cost_param_adapted,
                    double height_change_cost_param, bool only_yawed);
void findFreeDirections(
    const Histogram& histogram, double safety_radius,
    nav_msgs::GridCells& path_candidates, nav_msgs::GridCells& path_selected,
    nav_msgs::GridCells& path_rejected, nav_msgs::GridCells& path_blocked,
    const nav_msgs::GridCells& path_waypoints,
    std::vector<float>& cost_path_candidates, const Eigen::Vector3f& goal,
    const Eigen::Vector3f& position, const Eigen::Vector3f& position_old,
    double goal_cost_param, double smooth_cost_param,
    double height_change_cost_param_adapted, double height_change_cost_param,
    bool only_yawed, int resolution_alpha);
void printHistogram(Histogram hist, std::vector<int> z_FOV_idx, int e_FOV_min,
                    int e_FOV_max, int e_chosen, int z_chosen,
                    double resolution);
bool calculateCostMap(const std::vector<float>& cost_path_candidates,
                      std::vector<int>& cost_idx_sorted);
bool getDirectionFromTree(
    Eigen::Vector3f& p,
    const std::vector<geometry_msgs::Point>& path_node_positions,
    const Eigen::Vector3f& position);
}
#endif  // LOCAL_PLANNER_FUNCTIONS_H
