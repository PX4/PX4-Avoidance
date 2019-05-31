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

  float cell_size = grid.getCellSize();
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
  cell.scale.x = cell_size;
  cell.scale.y = cell_size;
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

  for (size_t i = 0; i < grid.getRowColSize(); i++) {
    for (size_t j = 0; j < grid.getRowColSize(); j++) {
      cell.pose.position.x = (i * cell_size) + grid_min.x() + (cell_size / 2.f);
      cell.pose.position.y = (j * cell_size) + grid_min.y() + (cell_size / 2.f);
      cell.pose.position.z = 0.0;

      float h = ((range_max - range_min) * (mean(i, j) - variance_min_value) /
                 (variance_max_value - variance_min_value)) +
                range_min;
      float red, green, blue;
      float max_aa = 1.f;
      std::tie(cell.color.r, cell.color.g, cell.color.b) = HSVtoRGB(std::make_tuple(h, 1.f, 1.f));

      marker_array.markers.push_back(cell);
      cell.id += 1;
    }
  }
  mean_pub_.publish(marker_array);
}
std::tuple<float, float, float> SafeLandingPlannerVisualization::HSVtoRGB(std::tuple<float, float, float> hsv) {

  std::tuple<float, float, float> rgb;
  float fC = std::get<2>(hsv) * std::get<1>(hsv); //fV * fS;  // Chroma
  float fHPrime = fmod(std::get<0>(hsv) / 60.0, 6);
  float fX = fC * (1 - fabs(fmod(fHPrime, 2) - 1));
  float fM = std::get<2>(hsv) - fC;

  if (0 <= fHPrime && fHPrime < 1) {
    std::get<0>(rgb) = fC;
    std::get<1>(rgb) = fX;
    std::get<2>(rgb) = 0;
  } else if (1 <= fHPrime && fHPrime < 2) {
    std::get<0>(rgb) = fX;
    std::get<1>(rgb) = fC;
    std::get<2>(rgb) = 0;
  } else if (2 <= fHPrime && fHPrime < 3) {
    std::get<0>(rgb) = 0;
    std::get<1>(rgb) = fC;
    std::get<2>(rgb) = fX;
  } else if (3 <= fHPrime && fHPrime < 4) {
    std::get<0>(rgb) = 0;
    std::get<1>(rgb) = fX;
    std::get<2>(rgb) = fC;
  } else if (4 <= fHPrime && fHPrime < 5) {
    std::get<0>(rgb) = fX;
    std::get<1>(rgb) = 0;
    std::get<2>(rgb) = fC;
  } else if (5 <= fHPrime && fHPrime < 6) {
    std::get<0>(rgb) = fC;
    std::get<1>(rgb) = 0;
    std::get<2>(rgb) = fX;
  } else {
    std::get<0>(rgb) = 0;
    std::get<1>(rgb) = 0;
    std::get<2>(rgb) = 0;
  }

  std::get<0>(rgb) += fM;
  std::get<1>(rgb) += fM;
  std::get<2>(rgb) += fM;

  return rgb;
}

void SafeLandingPlannerVisualization::publishStandardDeviation(
    const Grid& grid) {
  visualization_msgs::MarkerArray marker_array;

  float cell_size = grid.getCellSize();
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
  cell.scale.x = cell_size;
  cell.scale.y = cell_size;
  cell.scale.z = 0.1;
  cell.color.a = 0.5;

  Eigen::Vector2f grid_min, grid_max;
  grid.getGridLimits(grid_min, grid_max);

  Eigen::MatrixXf variance = grid.getVariance();

  float variance_max_value = 1.0f;
  float variance_min_value = 0.0f;
  float range_max = 360.f;
  float range_min = 0.f;

  for (size_t i = 0; i < grid.getRowColSize(); i++) {
    for (size_t j = 0; j < grid.getRowColSize(); j++) {
      cell.pose.position.x = (i * cell_size) + grid_min.x() + (cell_size / 2.f);
      cell.pose.position.y = (j * cell_size) + grid_min.y() + (cell_size / 2.f);
      cell.pose.position.z = 0.0;

      float h = ((range_max - range_min) *
                 (sqrtf(variance(i, j)) - variance_min_value) /
                 (variance_max_value - variance_min_value)) +
                range_min;

      std::tie(cell.color.r, cell.color.g, cell.color.b) = HSVtoRGB(std::make_tuple(h, 1.f, 1.f));

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

  float cell_size = grid.getCellSize();
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
  cell.scale.x = cell_size;
  cell.scale.y = cell_size;
  cell.scale.z = 0.1;
  cell.color.a = 0.5;
  Eigen::Vector2f grid_min, grid_max;
  grid.getGridLimits(grid_min, grid_max);
  int offset = grid.land_.rows() / 2;

  for (size_t i = 0; i < grid.getRowColSize(); i++) {
    for (size_t j = 0; j < grid.getRowColSize(); j++) {
      cell.pose.position.x = (i * cell_size) + grid_min.x() + (cell_size / 2.f);
      cell.pose.position.y = (j * cell_size) + grid_min.y() + (cell_size / 2.f);
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
