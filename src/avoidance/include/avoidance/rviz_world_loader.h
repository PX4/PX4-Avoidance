#ifndef RVIZ_WORLD_H
#define RVIZ_WORLD_H

#include <chrono>

#include <Eigen/Core>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "yaml-cpp/yaml.h"

#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <sys/stat.h>

namespace avoidance {

// data types
struct world_object {
  std::string type;
  std::string name;
  std::string frame_id;
  std::string mesh_resource;
  Eigen::Vector3f position;
  Eigen::Vector4f orientation;
  Eigen::Vector3f scale;
};

// yaml file extraction operators
void operator>>(const YAML::Node& node, Eigen::Vector3f& v);
void operator>>(const YAML::Node& node, Eigen::Vector4f& v);
void operator>>(const YAML::Node& node, world_object& item);

class WorldVisualizer : public rclcpp::Node
{
private:
  /**
   * @brief      helper function to resolve gazebo model path
   **/
  int resolveUri(std::string& uri) const;

  rclcpp::TimerBase::SharedPtr loop_timer_;

  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr pose_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr world_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr drone_pub_;

  std::string world_path_;

  void loopCallback();

public:
  /**
   * @brief     WorldVisualizer Node Class
   */
  WorldVisualizer();
  ~WorldVisualizer() = default;

  /**
   * @brief      callback for subscribing mav pose topic
   **/
  void positionCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) const;

  /**
   * @brief      parse the yaml file and publish world marker
   * @param[in]  world_path, path of the yaml file describing the world
   **/
  int visualizeRVIZWorld(const std::string& world_path);

  /**
   * @brief      visualize the drone mesh at the current drone position
   * @param[in]  pose, current drone pose
   **/
  int visualizeDrone(const px4_msgs::msg::VehicleOdometry& pose) const;
};
}

#endif  // RVIZ_WORLD_H
