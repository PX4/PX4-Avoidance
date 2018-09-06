#ifndef RVIZ_LOAD_H
#define RVIZ_LOAD_H


#include "yaml-cpp/yaml.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <sys/stat.h>

//data types
struct Vec3 {
   float x, y, z;
};

struct Vec4 {
   float x, y, z, w;
};

struct world_object{
   std::string type;
   std::string name;
   std::string frame_id;
   std::string mesh_resource;
   Vec3 position;
   Vec4 orientation;
   Vec3 scale;
};

//extraction operators for these types
void operator >> (const YAML::Node& node, Vec3& v);
void operator >> (const YAML::Node& node, Vec4& v);
void operator >> (const YAML::Node& node, world_object& item);

//helper function to resolve gazebo model paths
int resolveUri(std::string& uri);

//yaml parsing and marker publishing
int visualizeRVIZWorld(visualization_msgs::MarkerArray& marker_array, std::string world_path);

#endif  // RVIZ_LOAD_H
