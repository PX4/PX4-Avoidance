#ifndef RVIZ_WORLD_H
#define RVIZ_WORLD_H

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "yaml-cpp/yaml.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

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

class WorldVisualizer {
 private:
  /**
  * @brief      helper function to resolve gazebo model path
  **/
  int resolveUri(std::string& uri);

  ros::NodeHandle nh_;

  ros::Timer loop_timer_;

  ros::Subscriber pose_sub_;
  ros::Publisher world_pub_;
  ros::Publisher drone_pub_;

  std::string world_path_;
  std::string nodelet_ns_;

  void loopCallback(const ros::TimerEvent& event);

 public:
  WorldVisualizer(const ros::NodeHandle& nh, const std::string& nodelet_ns);

  /**
  * @brief      initializes all publishers used for local planner visualization
  * @param      nh, nodehandle to initialize publishers
  **/
  void initializePublishers(ros::NodeHandle& nh);

  /**
  * @brief      callback for subscribing mav pose topic
  **/
  void positionCallback(const geometry_msgs::PoseStamped& msg);

  /**
  * @brief      parse the yaml file and publish world marker
  * @param[in]  world_path, path of the yaml file describing the world
  **/
  int visualizeRVIZWorld(const std::string& world_path);

  /**
  * @brief      visualize the drone mesh at the current drone position
  * @param[in]  pose, current drone pose
  **/
  int visualizeDrone(const geometry_msgs::PoseStamped& pose);
};
}

#endif  // RVIZ_WORLD_H
