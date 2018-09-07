#ifndef RVIZ_WORLD_H
#define RVIZ_WORLD_H


#include "yaml-cpp/yaml.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <Eigen/Core>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <sys/stat.h>

//data types
struct world_object{
   std::string type;
   std::string name;
   std::string frame_id;
   std::string mesh_resource;
   Eigen::Vector3f position;
   Eigen::Vector4f orientation;
   Eigen::Vector3f scale;
};

//extraction operators
void operator >> (const YAML::Node& node, Eigen::Vector3f& v);
void operator >> (const YAML::Node& node, Eigen::Vector4f& v);
void operator >> (const YAML::Node& node, world_object& item);

//helper function to resolve gazebo model paths
int resolveUri(std::string& uri);

//yaml parsing and marker publishing
int visualizeRVIZWorld(visualization_msgs::MarkerArray& marker_array, std::string world_path);

#endif  // RVIZ_WORLD_H
