#include "safe_landing_planner/safe_landing_planner_visualization.hpp"

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace avoidance {

void SafeLandingPlannerVisualization::initializePublishers(
    ros::NodeHandle& nh) {
  local_pointcloud_pub_ =
      nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("/grid_pointcloud", 1);
  path_actual_pub_ =
      nh.advertise<visualization_msgs::Marker>("/path_actual", 1);
  grid_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/grid", 1);
  mean_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/grid_mean", 1);
  std_dev_pub_ =
      nh.advertise<visualization_msgs::MarkerArray>("/grid_std_dev", 1);
}

void SafeLandingPlannerVisualization::visualizeSafeLandingPlanner(
    const SafeLandingPlanner& planner, const geometry_msgs::Point& pos,
    const geometry_msgs::Point& last_pos) {
  local_pointcloud_pub_.publish(planner.visualization_cloud_);
  publishGrid(planner.getGrid(), pos, planner.getSmoothingSize());
  publishMean(planner.getGrid());
  publishStandardDeviation(planner.getGrid());
  publishPaths(pos, last_pos);
}

void SafeLandingPlannerVisualization::publishMean(const Grid& grid) {
  visualization_msgs::MarkerArray marker_array;

  visualization_msgs::Marker cell;
  cell.header.frame_id = "local_origin";
  cell.header.stamp = ros::Time::now();
  cell.id = 0;
  cell.type = visualization_msgs::Marker::CUBE;
  cell.action = visualization_msgs::Marker::ADD;
  cell.pose.orientation.x = 0.0;
  cell.pose.orientation.y = 0.0;
  cell.pose.orientation.z = 0.0;
  cell.pose.orientation.w = 1.0;
  cell.scale.x = grid.cell_size_;
  cell.scale.y = grid.cell_size_;
  cell.scale.z = 0.1;
  cell.color.a = 0.5;
  cell.color.r = 0.0;
  cell.color.g = 0.0;
  Eigen::Vector2f grid_min, grid_max;
  grid.getGridLimits(grid_min, grid_max);

  float variance_max_value = 10.0f;
  float variance_min_value = -1.0f;
  float range_max = 360.f;
  float range_min = 0.f;

  Eigen::MatrixXf mean = grid.getMean();

  for (size_t i = 0; i < std::ceil(grid.grid_size_ / grid.cell_size_); i++) {
    for (size_t j = 0; j < std::ceil(grid.grid_size_ / grid.cell_size_); j++) {
      cell.pose.position.x =
          (i * grid.cell_size_) + grid_min.x() + (grid.cell_size_ / 2.f);
      cell.pose.position.y =
          (j * grid.cell_size_) + grid_min.y() + (grid.cell_size_ / 2.f);
      cell.pose.position.z = 0.0;

      float h = ((range_max - range_min) * (mean(i, j) - variance_min_value) /
                 (variance_max_value - variance_min_value)) +
                range_min;
      float red, green, blue;
      float max_aa = 1.f;
      HSVtoRGB(red, green, blue, h, max_aa, max_aa);
      cell.color.r = red;
      cell.color.g = green;
      cell.color.b = blue;

      marker_array.markers.push_back(cell);
      cell.id += 1;
    }
  }
  mean_pub_.publish(marker_array);
}

void SafeLandingPlannerVisualization::HSVtoRGB(float& fR, float& fG, float& fB,
                                               float& fH, float& fS,
                                               float& fV) {
  float fC = fV * fS;  // Chroma
  float fHPrime = fmod(fH / 60.0, 6);
  float fX = fC * (1 - fabs(fmod(fHPrime, 2) - 1));
  float fM = fV - fC;

  if (0 <= fHPrime && fHPrime < 1) {
    fR = fC;
    fG = fX;
    fB = 0;
  } else if (1 <= fHPrime && fHPrime < 2) {
    fR = fX;
    fG = fC;
    fB = 0;
  } else if (2 <= fHPrime && fHPrime < 3) {
    fR = 0;
    fG = fC;
    fB = fX;
  } else if (3 <= fHPrime && fHPrime < 4) {
    fR = 0;
    fG = fX;
    fB = fC;
  } else if (4 <= fHPrime && fHPrime < 5) {
    fR = fX;
    fG = 0;
    fB = fC;
  } else if (5 <= fHPrime && fHPrime < 6) {
    fR = fC;
    fG = 0;
    fB = fX;
  } else {
    fR = 0;
    fG = 0;
    fB = 0;
  }

  fR += fM;
  fG += fM;
  fB += fM;
}

void SafeLandingPlannerVisualization::publishStandardDeviation(
    const Grid& grid) {
  visualization_msgs::MarkerArray marker_array;

  visualization_msgs::Marker cell;
  cell.header.frame_id = "local_origin";
  cell.header.stamp = ros::Time::now();
  cell.id = 0;
  cell.type = visualization_msgs::Marker::CUBE;
  cell.action = visualization_msgs::Marker::ADD;
  cell.pose.orientation.x = 0.0;
  cell.pose.orientation.y = 0.0;
  cell.pose.orientation.z = 0.0;
  cell.pose.orientation.w = 1.0;
  cell.scale.x = grid.cell_size_;
  cell.scale.y = grid.cell_size_;
  cell.scale.z = 0.1;
  cell.color.a = 0.5;

  Eigen::Vector2f grid_min, grid_max;
  grid.getGridLimits(grid_min, grid_max);

  Eigen::MatrixXf variance = grid.getVariance();

  float variance_max_value = 1.0f;
  float variance_min_value = 0.0f;
  float range_max = 360.f;
  float range_min = 0.f;

  for (size_t i = 0; i < std::ceil(grid.grid_size_ / grid.cell_size_); i++) {
    for (size_t j = 0; j < std::ceil(grid.grid_size_ / grid.cell_size_); j++) {
      cell.pose.position.x =
          (i * grid.cell_size_) + grid_min.x() + (grid.cell_size_ / 2.f);
      cell.pose.position.y =
          (j * grid.cell_size_) + grid_min.y() + (grid.cell_size_ / 2.f);
      cell.pose.position.z = 0.0;

      float h = ((range_max - range_min) *
                 (sqrtf(variance(i, j)) - variance_min_value) /
                 (variance_max_value - variance_min_value)) +
                range_min;
      float red, green, blue;
      float max_aa = 1.f;
      HSVtoRGB(red, green, blue, h, max_aa, max_aa);
      cell.color.r = red;
      cell.color.g = green;
      cell.color.b = blue;

      marker_array.markers.push_back(cell);
      cell.id += 1;
    }
  }
  std_dev_pub_.publish(marker_array);
}

void SafeLandingPlannerVisualization::publishGrid(
    const Grid& grid, const geometry_msgs::Point& pos,
    float smoothing_size) const {
  visualization_msgs::MarkerArray marker_array;

  visualization_msgs::Marker cell;
  cell.header.frame_id = "local_origin";
  cell.header.stamp = ros::Time::now();
  cell.id = 0;
  cell.type = visualization_msgs::Marker::CUBE;
  cell.action = visualization_msgs::Marker::ADD;
  cell.pose.orientation.x = 0.0;
  cell.pose.orientation.y = 0.0;
  cell.pose.orientation.z = 0.0;
  cell.pose.orientation.w = 1.0;
  cell.scale.x = grid.cell_size_;
  cell.scale.y = grid.cell_size_;
  cell.scale.z = 0.1;
  cell.color.a = 0.5;
  Eigen::Vector2f grid_min, grid_max;
  grid.getGridLimits(grid_min, grid_max);
  int offset = grid.land_.rows() / 2;

  for (size_t i = 0; i < std::ceil(grid.grid_size_ / grid.cell_size_); i++) {
    for (size_t j = 0; j < std::ceil(grid.grid_size_ / grid.cell_size_); j++) {
      cell.pose.position.x =
          (i * grid.cell_size_) + grid_min.x() + (grid.cell_size_ / 2.f);
      cell.pose.position.y =
          (j * grid.cell_size_) + grid_min.y() + (grid.cell_size_ / 2.f);
      cell.pose.position.z = 0.0;
      if (grid.land_(i, j)) {
        cell.color.r = 0.0;
        cell.color.g = 1.0;
        cell.color.b = 0.0;
      } else {
        cell.color.r = 1.0;
        cell.color.g = 0.0;
        cell.color.b = 0.0;
      }
      cell.scale.z = 0.1;

      if (i >= (offset - smoothing_size) && i < (offset + smoothing_size) &&
          j >= (offset - smoothing_size) && j < (offset + smoothing_size)) {
        cell.scale.z = 0.8;
      }
      marker_array.markers.push_back(cell);
      cell.id += 1;
    }
  }
  grid_pub_.publish(marker_array);
}

void SafeLandingPlannerVisualization::publishPaths(
    const geometry_msgs::Point& pos, const geometry_msgs::Point& last_pos) {
  visualization_msgs::Marker path_actual_marker;
  path_actual_marker.header.frame_id = "local_origin";
  path_actual_marker.header.stamp = ros::Time::now();
  path_actual_marker.id = path_length_;
  path_actual_marker.type = visualization_msgs::Marker::LINE_STRIP;
  path_actual_marker.action = visualization_msgs::Marker::ADD;
  path_actual_marker.pose.orientation.w = 1.0;
  path_actual_marker.scale.x = 0.03;
  path_actual_marker.color.a = 1.0;
  path_actual_marker.color.r = 0.0;
  path_actual_marker.color.g = 1.0;
  path_actual_marker.color.b = 0.0;

  path_actual_marker.points.push_back(last_pos);
  path_actual_marker.points.push_back(pos);
  path_actual_pub_.publish(path_actual_marker);
  path_length_ += 1;
}
}
