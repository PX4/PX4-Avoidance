#ifndef GLOBAL_PLANNER_COMMON_ROS_H_
#define GLOBAL_PLANNER_COMMON_ROS_H_

#include <string>

#include <math.h>  // sqrt
#include <nav_msgs/msg/path.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/transform_listener.h>  // getYaw createQuaternionMsgFromYaw
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "global_planner/common.h"  // hasSameYawAndAltitude

// This file contains general functions which have some Ros dependancy

namespace global_planner {

// GLOBAL PLANNER

template <typename P>
tf2::Vector3 toTfVector3(const P& point) {
  return tf2::Vector3(point.x, point.y, point.z);
}

inline double distance(const geometry_msgs::msg::PoseStamped& a, const geometry_msgs::msg::PoseStamped& b) {
  return distance(a.pose.position, b.pose.position);
}

// https://github.com/trainman419/fiducials_ros/blob/master/src/fiducials_localization.cpp
inline geometry_msgs::msg::TwistStamped transformTwistMsg(const tf2_ros::TransformListener& listener,
                                                          const std::string& target_frame,
                                                          const std::string& fixed_frame,
                                                          const geometry_msgs::msg::TwistStamped& msg) {
  auto transformed_msg = msg;
  geometry_msgs::msg::Vector3Stamped before;
  before.vector = msg.twist.linear;
  before.header = msg.header;
  geometry_msgs::msg::Vector3Stamped after;
  // TODO :: apply tf2 transformVector
  // listener.transformVector(target_frame, rclcpp::Time(0), before, fixed_frame, after);
  transformed_msg.twist.linear = after.vector;
  return transformed_msg;
}

// Returns a spectral color between red (0.0) and blue (1.0)
inline std_msgs::msg::ColorRGBA spectralColor(double hue, double alpha = 1.0) {
  std_msgs::msg::ColorRGBA color;
  color.r = std::max(0.0, 2 * hue - 1);
  color.g = 1.0 - 2.0 * std::abs(hue - 0.5);
  color.b = std::max(0.0, 1.0 - 2 * hue);
  color.a = alpha;
  return color;
}

template <typename Point, typename Color>
visualization_msgs::msg::Marker createMarker(int id, Point position, Color color, double scale = 0.1,
                                             std::string frame_id = "/world") {
  visualization_msgs::msg::Marker marker;
  marker.id = id;
  marker.header.frame_id = frame_id;
  marker.header.stamp = rclcpp::Time();
  marker.pose.position = position;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = marker.scale.y = marker.scale.z = scale;
  marker.color = color;
  return marker;
}

// double distance(const Eigen::Vector3d & a, const Eigen::Vector3d & b) {
//   double xDiff = a[0] - b[0];
//   double yDiff = a[1] - b[1];
//   double zDiff = a[2] - b[2];
//   return sqrt(squared(xDiff) + squared(yDiff) + squared(zDiff));
// }

// Returns true if msg1 and msg2 have both the same altitude and orientation
inline bool hasSameYawAndAltitude(const geometry_msgs::msg::Pose& msg1, const geometry_msgs::msg::Pose& msg2) {
  return msg1.orientation.z == msg2.orientation.z && msg1.orientation.w == msg2.orientation.w &&
         msg1.position.z == msg2.position.z;
}

inline double pathLength(const nav_msgs::msg::Path& path) {
  double total_dist = 0.0;
  for (int i = 1; i < path.poses.size(); ++i) {
    total_dist += distance(path.poses[i - 1], path.poses[i]);
  }
  return total_dist;
}

// Returns a path with only the corner points of msg
inline std::vector<geometry_msgs::msg::PoseStamped> filterPathCorners(
    const std::vector<geometry_msgs::msg::PoseStamped>& msg) {
  std::vector<geometry_msgs::msg::PoseStamped> corners = msg;
  corners.clear();
  if (msg.size() < 1) {
    return corners;
  }

  int n = msg.size();
  corners.push_back(msg.front());
  for (int i = 1; i < n - 1; ++i) {
    geometry_msgs::msg::Point last = msg[i - 1].pose.position;
    geometry_msgs::msg::Point curr = msg[i].pose.position;
    geometry_msgs::msg::Point next = msg[i + 1].pose.position;
    bool same_x = (next.x - curr.x) == (curr.x - last.x);
    bool same_y = (next.y - curr.y) == (curr.y - last.y);
    bool same_z = (next.z - curr.z) == (curr.z - last.z);
    if (!(same_x && same_y && same_z)) {
      corners.push_back(msg[i]);
    }
  }
  corners.push_back(msg.back());
  return corners;
}

inline double pathKineticEnergy(const nav_msgs::msg::Path& path) {
  if (path.poses.size() < 3) {
    return 0.0;
  }
  std::vector<double> vel_x;
  std::vector<double> vel_y;
  std::vector<double> vel_z;
  for (int i = 1; i < path.poses.size(); ++i) {
    vel_x.push_back(path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x);
    vel_y.push_back(path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y);
    vel_z.push_back(path.poses[i].pose.position.z - path.poses[i - 1].pose.position.z);
  }

  double total_energy = 0.0;
  for (int i = 1; i < vel_x.size(); ++i) {
    total_energy += std::abs(vel_x[i] * vel_x[i] - vel_x[i - 1] * vel_x[i - 1]);
    total_energy += std::abs(vel_y[i] * vel_y[i] - vel_y[i - 1] * vel_y[i - 1]);
    total_energy += std::abs(vel_z[i] * vel_z[i] - vel_z[i - 1] * vel_z[i - 1]);
  }
  return total_energy;
}

inline double pathEnergy(const nav_msgs::msg::Path& path, double up_penalty) {
  double total_energy = 0.0;
  for (int i = 1; i < path.poses.size(); ++i) {
    total_energy += distance(path.poses[i - 1], path.poses[i]);
    double altitude_increase = path.poses[i].pose.position.z - path.poses[i - 1].pose.position.z;
    total_energy += std::max(0.0, up_penalty * altitude_increase);
  }
  return total_energy;
}

}  // namespace global_planner

#endif /* GLOBAL_PLANNER_COMMON_ROS_H_ */