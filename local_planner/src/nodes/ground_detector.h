#ifndef GROUND_DETECTOR_H
#define GROUND_DETECTOR_H

#include <math.h>
#include <vector>
#include "box.h"
#include "common.h"
#include "histogram.h"
#include "planner_functions.h"

#include <math.h>
#include <Eigen/Dense>
#include <deque>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <ros/ros.h>

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

#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <nav_msgs/GridCells.h>
#include <nav_msgs/Path.h>

#include <dynamic_reconfigure/server.h>
#include <local_planner/LocalPlannerNodeConfig.h>

class GroundDetector {
  pcl::PointCloud<pcl::PointXYZ> ground_cloud_;

  double min_cloud_size_ = 160.0;

  std::vector<double> ground_heights_;
  std::vector<double> ground_xmax_;
  std::vector<double> ground_xmin_;
  std::vector<double> ground_ymax_;
  std::vector<double> ground_ymin_;

  double ground_inlier_distance_threshold_;
  double ground_inlier_angle_threshold_;
  double min_flight_height_;
  double begin_rise_;

  double ground_dist_;
  double min_plane_points_ = 100.0;
  double min_plane_percentage_ = 0.6;

  bool over_obstacle_ = false;
  bool is_near_min_height_ = false;
  bool too_low_ = false;

  geometry_msgs::Point closest_point_on_ground_;
  geometry_msgs::Quaternion ground_orientation_;
  geometry_msgs::PoseStamped pose_;

 public:
  bool ground_detected_;
  Box ground_box_;
  Box ground_box_size_;

  double min_dist_to_ground_;

  GroundDetector();
  GroundDetector(const GroundDetector &detector);
  ~GroundDetector();

  void detectGround(pcl::PointCloud<pcl::PointXYZ> &complete_cloud);
  void setParams(double min_dist_to_ground, double min_cloud_size);
  void setPose(geometry_msgs::PoseStamped pose);
  void fitPlane();
  void dynamicReconfigureSetGroundParams(
      avoidance::LocalPlannerNodeConfig &config, uint32_t level);
  void logData(std::string log_name);
  int getMinFlightElevationIndex(geometry_msgs::PoseStamped current_pose,
                                 double min_flight_height, int resolution);
  double getMinFlightHeight(geometry_msgs::PoseStamped current_pose,
                            geometry_msgs::TwistStamped curr_vel,
                            bool over_obstacle_old,
                            double min_flight_height_old, double margin_old);
  void getHeightInformation(bool &over_obstacle, bool &too_low,
                            bool &is_near_min_height);
  double getMargin();
  void getGroundCloudForVisualization(
      pcl::PointCloud<pcl::PointXYZ> &ground_cloud);
  void getGroundDataForVisualization(
      geometry_msgs::Point &closest_point_on_ground,
      geometry_msgs::Quaternion &ground_orientation,
      std::vector<double> &ground_heights, std::vector<double> &ground_xmax,
      std::vector<double> &ground_xmin, std::vector<double> &ground_ymax,
      std::vector<double> &ground_ymin);
  void initializeGroundBox(double min_dist_to_ground);
  void reset();
};

#endif  // GROUND_DETECTOR_H
