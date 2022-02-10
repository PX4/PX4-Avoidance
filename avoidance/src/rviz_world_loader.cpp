#include "avoidance/rviz_world_loader.h"

#include <ros/console.h>

namespace avoidance {

WorldVisualizer::WorldVisualizer(const ros::NodeHandle& nh, const std::string& nodelet_ns)
    : nh_(nh), nodelet_ns_(nodelet_ns) {
  pose_sub_ = nh_.subscribe<const geometry_msgs::PoseStamped&>("mavros/local_position/pose", 1,
                                                               &WorldVisualizer::positionCallback, this);

  world_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("world", 1);
  drone_pub_ = nh_.advertise<visualization_msgs::Marker>("drone", 1);
  loop_timer_ = nh_.createTimer(ros::Duration(2.0), &WorldVisualizer::loopCallback, this);

  std::string world_string;
  if (!nodelet_ns_.empty()) {
    world_string = nodelet_ns_ + "/world_name";
  } else {
    world_string = "world_name";
  }
  nh_.param<std::string>(world_string, world_path_, "");
}

void WorldVisualizer::loopCallback(const ros::TimerEvent& event) {
  // visualize world in RVIZ
  if (!world_path_.empty()) {
    if (visualizeRVIZWorld(world_path_)) ROS_WARN("[WorldVisualizer] Failed to visualize Rviz world");
  }
}

int WorldVisualizer::resolveUri(std::string& uri) {
  // Iterate through all locations in GAZEBO_MODEL_PATH
  char* gazebo_model_path = getenv("GAZEBO_MODEL_PATH");
  char* home = getenv("HOME");
  uri = uri.substr(7, std::string::npos);
  std::stringstream all_locations(gazebo_model_path, std::ios_base::app | std::ios_base::out | std::ios_base::in);
  all_locations << ":" << home << "/.gazebo/models";
  std::string current_location;
  while (getline(all_locations, current_location, ':')) {
    struct stat s;
    std::string temp = current_location + uri;
    if (stat(temp.c_str(), &s) == 0) {
      if (s.st_mode & S_IFREG)  // this path describes a file
      {
        uri = "file://" + current_location + uri;
        return 0;
      }
    }
  }
  return 1;
}

int WorldVisualizer::visualizeRVIZWorld(const std::string& world_path) {
  std::ifstream fin(world_path);
  YAML::Node doc = YAML::Load(fin);
  size_t object_counter = 0;
  visualization_msgs::MarkerArray marker_array;

  for (YAML::const_iterator it = doc.begin(); it != doc.end(); ++it) {
    const YAML::Node& node = *it;
    world_object item;
    node >> item;
    object_counter++;

    // convert object to marker
    visualization_msgs::Marker m;
    m.header.frame_id = item.frame_id;
    m.header.stamp = ros::Time::now();

    if (item.type == "mesh") {
      if (item.mesh_resource.find("model://") != std::string::npos) {
        if (resolveUri(item.mesh_resource)) {
          ROS_ERROR("RVIZ world loader could not find model");
          return 1;
        }
      }
      m.type = visualization_msgs::Marker::MESH_RESOURCE;
      m.mesh_resource = item.mesh_resource;
      m.mesh_use_embedded_materials = true;
    } else if (item.type == "cube") {
      m.type = visualization_msgs::Marker::CUBE;
      m.color.a = 0.9;
      m.color.r = 0.5;
      m.color.g = 0.5;
      m.color.b = 0.5;
    } else if (item.type == "sphere") {
      m.type = visualization_msgs::Marker::SPHERE;
      m.color.a = 0.9;
      m.color.r = 0.5;
      m.color.g = 0.5;
      m.color.b = 0.5;
    } else if (item.type == "cylinder") {
      m.type = visualization_msgs::Marker::CYLINDER;
      m.color.a = 0.9;
      m.color.r = 0.5;
      m.color.g = 0.5;
      m.color.b = 0.5;
    } else {
      ROS_ERROR("RVIZ world loader invalid object type in yaml file");
      return 1;
    }

    m.scale.x = item.scale.x();
    m.scale.y = item.scale.y();
    m.scale.z = item.scale.z();
    m.pose.position.x = item.position.x();
    m.pose.position.y = item.position.y();
    m.pose.position.z = item.position.z();
    m.pose.orientation.x = item.orientation.x();
    m.pose.orientation.y = item.orientation.y();
    m.pose.orientation.z = item.orientation.z();
    m.pose.orientation.w = item.orientation.w();
    m.id = object_counter;
    m.lifetime = ros::Duration();
    m.action = visualization_msgs::Marker::ADD;
    marker_array.markers.push_back(m);
  }

  if (object_counter != marker_array.markers.size()) {
    ROS_ERROR("Could not display all world objects");
  }

  world_pub_.publish(marker_array);
  ROS_INFO_ONCE("Successfully loaded rviz world");
  return 0;
}

int WorldVisualizer::visualizeDrone(const geometry_msgs::PoseStamped& pose) {
  visualization_msgs::Marker drone;
  drone.header.frame_id = "local_origin";
  drone.header.stamp = ros::Time::now();
  drone.type = visualization_msgs::Marker::MESH_RESOURCE;
  drone.mesh_resource = "model://matrice_100/meshes/Matrice_100.dae";
  if (drone.mesh_resource.find("model://") != std::string::npos) {
    if (resolveUri(drone.mesh_resource)) {
      ROS_ERROR("RVIZ world loader could not find drone model");
      return 1;
    }
  }
  drone.mesh_use_embedded_materials = true;
  drone.scale.x = 1.5;
  drone.scale.y = 1.5;
  drone.scale.z = 1.5;
  drone.pose.position.x = pose.pose.position.x;
  drone.pose.position.y = pose.pose.position.y;
  drone.pose.position.z = pose.pose.position.z;
  drone.pose.orientation.x = pose.pose.orientation.x;
  drone.pose.orientation.y = pose.pose.orientation.y;
  drone.pose.orientation.z = pose.pose.orientation.z;
  drone.pose.orientation.w = pose.pose.orientation.w;
  drone.id = 0;
  drone.lifetime = ros::Duration();
  drone.action = visualization_msgs::Marker::ADD;

  drone_pub_.publish(drone);

  return 0;
}

void WorldVisualizer::positionCallback(const geometry_msgs::PoseStamped& msg) {
  // visualize drone in RVIZ
  if (!world_path_.empty()) {
    if (visualizeDrone(msg)) {
      ROS_WARN("Failed to visualize drone in RViz");
    }
  }
}

// extraction operators
void operator>>(const YAML::Node& node, Eigen::Vector3f& v) {
  v.x() = node[0].as<float>();
  v.y() = node[1].as<float>();
  v.z() = node[2].as<float>();
}

void operator>>(const YAML::Node& node, Eigen::Vector4f& v) {
  v.x() = node[0].as<float>();
  v.y() = node[1].as<float>();
  v.z() = node[2].as<float>();
  v.w() = node[3].as<float>();
}

void operator>>(const YAML::Node& node, world_object& item) {
  item.type = node["type"].as<std::string>();
  item.name = node["name"].as<std::string>();
  item.frame_id = node["frame_id"].as<std::string>();
  item.mesh_resource = node["mesh_resource"].as<std::string>();
  node["position"] >> item.position;
  node["orientation"] >> item.orientation;
  node["scale"] >> item.scale;
}
}
