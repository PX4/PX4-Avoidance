#ifndef LOCAL_PLANNER_FUNCTIONS_H
#define LOCAL_PLANNER_FUNCTIONS_H

#include <math.h>
#include <Eigen/Dense>
#include <deque>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <tf/transform_listener.h>

#include "box.h"
#include "common.h"
#include "histogram.h"

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <nav_msgs/GridCells.h>
#include <nav_msgs/Path.h>

void initGridCells(nav_msgs::GridCells *cell);
void calculateSphere(geometry_msgs::Point &sphere_center, int &sphere_age,
                     geometry_msgs::Point temp_centerpoint,
                     int counter_sphere_points, double sphere_speed);
double adaptSafetyMarginHistogram(double dist_to_closest_point,
                                  double cloud_size, double min_cloud_size);
void filterPointCloud(pcl::PointCloud<pcl::PointXYZ> &cropped_cloud,
                      geometry_msgs::Point &closest_point,
                      geometry_msgs::Point &temp_sphere_center,
                      double &distance_to_closest_point, int &counter_backoff,
                      int &counter_sphere,
                      pcl::PointCloud<pcl::PointXYZ> complete_cloud,
                      double min_cloud_size, double min_dist_backoff,
                      double sphere_radius, Box histogram_box,
                      geometry_msgs::Point position, double min_realsense_dist);
void calculateFOV(std::vector<int> &z_FOV_idx, int &e_FOV_min, int &e_FOV_max,
                  double yaw, double pitch);
void propagateHistogram(Histogram &polar_histogram_est,
                        pcl::PointCloud<pcl::PointXYZ> reprojected_points,
                        std::vector<double> reprojected_points_age,
                        std::vector<double> reprojected_points_dist,
                        geometry_msgs::PoseStamped position);
void generateNewHistogram(Histogram &polar_histogram,
                          pcl::PointCloud<pcl::PointXYZ> cropped_cloud,
                          geometry_msgs::PoseStamped position);
void combinedHistogram(bool &hist_empty, Histogram &new_hist,
                       Histogram propagated_hist, bool waypoint_outside_FOV,
                       std::vector<int> z_FOV_idx, int e_FOV_min,
                       int e_FOV_max);
void compressHistogramElevation(Histogram &new_hist, Histogram input_hist);
double costFunction(int e, int z, nav_msgs::GridCells path_waypoints,
                    geometry_msgs::Point goal,
                    geometry_msgs::PoseStamped position,
                    geometry_msgs::Point position_old, double goal_cost_param,
                    double smooth_cost_param,
                    double height_change_cost_param_adapted,
                    double height_change_cost_param, bool only_yawed);
void findFreeDirections(
    const Histogram &histogram, double safety_radius,
    nav_msgs::GridCells &path_candidates, nav_msgs::GridCells &path_selected,
    nav_msgs::GridCells &path_rejected, nav_msgs::GridCells &path_blocked,
    nav_msgs::GridCells &path_ground, nav_msgs::GridCells path_waypoints,
    std::vector<float> &cost_path_candidates, const geometry_msgs::Point &goal,
    const geometry_msgs::PoseStamped &position, const geometry_msgs::Point &position_old,
    double goal_cost_param, double smooth_cost_param,
    double height_change_cost_param_adapted, double height_change_cost_param,
    int e_min_idx, bool over_obstacle, bool only_yawed, int resolution_alpha);
void printHistogram(Histogram hist, std::vector<int> z_FOV_idx, int e_FOV_min,
                    int e_FOV_max, int e_chosen, int z_chosen,
                    double resolution);
bool calculateCostMap(std::vector<float> cost_path_candidates,
                      std::vector<int> &cost_idx_sorted);
bool getDirectionFromTree(geometry_msgs::Point &p,
                          std::vector<geometry_msgs::Point> path_node_positions,
                          geometry_msgs::Point position,
                          geometry_msgs::Point goal);
geometry_msgs::Point getSphereAdaptedWaypoint(
    geometry_msgs::Point position, geometry_msgs::Point wp,
    geometry_msgs::Point avoid_centerpoint, double avoid_radius);

#endif  // LOCAL_PLANNER_FUNCTIONS_H
