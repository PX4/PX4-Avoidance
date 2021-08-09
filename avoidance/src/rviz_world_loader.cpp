#include "avoidance/rviz_world_loader.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace avoidance {

WorldVisualizer::WorldVisualizer()
    : Node("world_visualizer"),
      world_path_(this->declare_parameter("world_path", ""))
{
  world_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/world", 1);
  drone_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/drone", 1);
  loop_timer_ = this->create_wall_timer(2s, std::bind(&WorldVisualizer::loopCallback, this));
  visualize_drone_timer_ = this->create_wall_timer(200ms, std::bind(&WorldVisualizer::visualizeDrone, this));

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  this->get_parameter("world_path", world_path_);
}

void WorldVisualizer::loopCallback() {
  // visualize world in RVIZ
  if (!world_path_.empty()) {
    if (visualizeRVIZWorld(world_path_)) RCLCPP_WARN(this->get_logger(), "[WorldVisualizer] Failed to visualize Rviz world");
  }
}

int WorldVisualizer::resolveUri(std::string& uri) const {
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
  visualization_msgs::msg::MarkerArray marker_array;

  for (YAML::const_iterator it = doc.begin(); it != doc.end(); ++it) {
    const YAML::Node& node = *it;
    world_object item;
    node >> item;
    object_counter++;

    // convert object to marker
    visualization_msgs::msg::Marker m;
    m.header.frame_id = item.frame_id;
    m.header.stamp = this->now();

    if (item.type == "mesh") {
      if (item.mesh_resource.find("model://") != std::string::npos) {
        if (resolveUri(item.mesh_resource)) {
          RCLCPP_ERROR(this->get_logger(), "RVIZ world loader could not find model");
          return 1;
        }
      }
      m.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
      m.mesh_resource = item.mesh_resource;
      m.mesh_use_embedded_materials = true;
    } else if (item.type == "cube") {
      m.type = visualization_msgs::msg::Marker::CUBE;
      m.color.a = 0.9;
      m.color.r = 0.5;
      m.color.g = 0.5;
      m.color.b = 0.5;
    } else if (item.type == "sphere") {
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.color.a = 0.9;
      m.color.r = 0.5;
      m.color.g = 0.5;
      m.color.b = 0.5;
    } else if (item.type == "cylinder") {
      m.type = visualization_msgs::msg::Marker::CYLINDER;
      m.color.a = 0.9;
      m.color.r = 0.5;
      m.color.g = 0.5;
      m.color.b = 0.5;
    } else {
      RCLCPP_ERROR(this->get_logger(), "RVIZ world loader invalid object type in yaml file");
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
    m.lifetime = rclcpp::Duration(0);
    m.action = visualization_msgs::msg::Marker::ADD;
    marker_array.markers.push_back(m);
  }

  if (object_counter != marker_array.markers.size()) {
    RCLCPP_ERROR(this->get_logger(), "Could not display all world objects");
  }

  world_pub_->publish(marker_array);
  RCLCPP_INFO_ONCE(this->get_logger(), "Successfully loaded rviz world");
  return 0;
}

void WorldVisualizer::visualizeDrone(){
  tf_buffer_->waitForTransform("base_frame", "local_origin_odom", rclcpp::Clock().now(), std::chrono::milliseconds(500),
    std::bind(&WorldVisualizer::tf2Callback, this, _1));
}

void WorldVisualizer::tf2Callback(const std::shared_future<geometry_msgs::msg::TransformStamped>& tf) {
  try {
    geometry_msgs::msg::TransformStamped transformStamped = tf.get();

    auto drone = visualization_msgs::msg::Marker();
    drone.header.frame_id = "base_frame";
    drone.header.stamp = rclcpp::Clock().now();
    drone.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    drone.mesh_resource = "model://matrice_100/meshes/Matrice_100.dae";
    if (drone.mesh_resource.find("model://") != std::string::npos) {
      if (resolveUri(drone.mesh_resource)) {
        RCLCPP_ERROR(this->get_logger(), "RVIZ world loader could not find drone model");
        return;
      }
    }
    drone.mesh_use_embedded_materials = true;
    drone.scale.x = 1.5;
    drone.scale.y = 1.5;
    drone.scale.z = 1.5;
    drone.pose.position.x = transformStamped.transform.translation.x;
    drone.pose.position.y = transformStamped.transform.translation.y;
    drone.pose.position.z = transformStamped.transform.translation.z;
    drone.pose.orientation.x = transformStamped.transform.rotation.x;
    drone.pose.orientation.y = transformStamped.transform.rotation.y;
    drone.pose.orientation.z = transformStamped.transform.rotation.z;
    drone.pose.orientation.w = transformStamped.transform.rotation.w;
    drone.color.r = 0.0f;
    drone.color.g = 1.0f;
    drone.color.b = 0.0f;
    drone.color.a = 1.0;
    drone.id = 0;
    drone.lifetime = rclcpp::Duration(0);
    drone.action = visualization_msgs::msg::Marker::ADD;

    drone_pub_->publish(drone);
  } catch(tf2::TimeoutException const& ex) {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
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
