/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef GLOBAL_PLANNER_COMMON_H_
#define GLOBAL_PLANNER_COMMON_H_

#include <string>

#include <nav_msgs/Path.h>
#include <tf/transform_listener.h> // getYaw createQuaternionMsgFromYaw 
#include <geometry_msgs/PoseStamped.h>

namespace avoidance {

// GLOBAL PLANNER

template <typename T>
double squared(T x) {
  return x * x;
}

// Returns Map[key] if it exists, default_val otherwise
template <typename Key, typename Value, typename Map>
Value getWithDefault(Map & m, const Key & key, const Value & default_val) {
  if (m.find(key) != m.end()) {
    return m[key];
  }
  return default_val;
}

template <typename P>
tf::Vector3 toTfVector3(P point) {
  return tf::Vector3(point.x, point.y, point.z);
}

geometry_msgs::TwistStamped transformTwistMsg(const tf::TransformListener & listener,
                                              const std::string & target_frame,
                                              const std::string & fixed_frame,
                                              const geometry_msgs::TwistStamped & msg) {

  auto transformed_msg = msg;
  geometry_msgs::Vector3Stamped before;
  before.vector = msg.twist.linear;
  before.header = msg.header;
  geometry_msgs::Vector3Stamped after;
  listener.transformVector(target_frame, ros::Time(0), before, fixed_frame, after);
  transformed_msg.twist.linear = after.vector;
  return transformed_msg;
}

double distance(geometry_msgs::PoseStamped & a, geometry_msgs::PoseStamped & b) {
  double diffX = a.pose.position.x - b.pose.position.x;
  double diffY = a.pose.position.y - b.pose.position.y;
  double diffZ = a.pose.position.z - b.pose.position.z;
  return sqrt(squared(diffX) + squared(diffY) + squared(diffZ));
}

double norm(geometry_msgs::Vector3 v) {
  return std::sqrt(squared(v.x) + squared(v.y) + squared(v.z));
}

// Returns a weighted average of start and end, where ratio is the weight of start 
double interpolate(double start, double end, double ratio) {
  return start + (end - start) * ratio;
}

// returns angle in the range [-pi, pi]
double angleToRange(double angle) {
  angle += M_PI;
  angle -= (2*M_PI) * std::floor( angle / (2*M_PI) );
  angle -= M_PI;
  return angle;
}

// double distance(const Eigen::Vector3d & a, const Eigen::Vector3d & b) {
//   double xDiff = a[0] - b[0];
//   double yDiff = a[1] - b[1];
//   double zDiff = a[2] - b[2];
//   return sqrt(squared(xDiff) + squared(yDiff) + squared(zDiff));
// }

// Returns true if msg1 and msg2 have both the same altitude and orientation
bool hasSameYawAndAltitude(const geometry_msgs::Pose& msg1,
                           const geometry_msgs::Pose& msg2) {

  return msg1.orientation.z == msg2.orientation.z
      && msg1.orientation.w == msg2.orientation.w
      && msg1.position.z == msg2.position.z;
}

template <typename T>
T rotateToWorldCoordinates(T point) {
  T newPoint;
  newPoint.x = point.y;
  newPoint.y = -point.x;
  newPoint.z = point.z;
  return newPoint;
}

template <typename T>
T rotateToMavrosCoordinates(T point) {
  T newPoint;
  newPoint.x = -point.y;
  newPoint.y = point.x;
  newPoint.z = point.z;
  return newPoint;
}

geometry_msgs::PoseStamped rotatePoseMsgToWorld(const geometry_msgs::PoseStamped & msg) {
  geometry_msgs::PoseStamped rot_msg = msg;
  rot_msg.pose.position = rotateToWorldCoordinates(msg.pose.position);
  double yaw = tf::getYaw(msg.pose.orientation);
  rot_msg.pose.orientation = tf::createQuaternionMsgFromYaw(yaw - M_PI/2);
  return rot_msg;
}

geometry_msgs::PoseStamped rotatePoseMsgToMavros(const geometry_msgs::PoseStamped & msg) {
  geometry_msgs::PoseStamped rot_msg = msg;
  rot_msg.pose.position = rotateToMavrosCoordinates(msg.pose.position);
  double yaw = tf::getYaw(msg.pose.orientation);
  rot_msg.pose.orientation = tf::createQuaternionMsgFromYaw(yaw + M_PI/2);
  return rot_msg;
}

double posterior(double p, double prior) {
  // p and prior are independent measurements of the same event
  double prob_obstacle = p * prior;
  double prob_free = (1-p) * (1-prior);
  return prob_obstacle / (prob_obstacle + prob_free+0.0001);
}

double pathLength(nav_msgs::Path & path) {
  double total_dist = 0.0;
  for (int i=1; i < path.poses.size(); ++i) {
    total_dist += distance(path.poses[i-1], path.poses[i]);
  }
  return total_dist;
}

// Returns a path with only the corner points of msg
std::vector<geometry_msgs::PoseStamped> filterPathCorners(const std::vector<geometry_msgs::PoseStamped>& msg) {
  std::vector<geometry_msgs::PoseStamped> corners = msg;
  corners.clear();
  if (msg.size() < 1) {
    return corners;
  }

  int n = msg.size();
  corners.push_back(msg.front());
  for (int i=1; i < n-1; ++i) {
    geometry_msgs::Point last = msg[i-1].pose.position;
    geometry_msgs::Point curr = msg[i].pose.position;
    geometry_msgs::Point next = msg[i+1].pose.position;
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


double pathKineticEnergy(nav_msgs::Path & path) {
  if (path.poses.size() < 3) {
    return 0.0;
  }
  std::vector<double> vel_x;
  std::vector<double> vel_y;
  std::vector<double> vel_z;
  for (int i=1; i < path.poses.size(); ++i) {
    vel_x.push_back(path.poses[i].pose.position.x - path.poses[i-1].pose.position.x);
    vel_y.push_back(path.poses[i].pose.position.y - path.poses[i-1].pose.position.y);
    vel_z.push_back(path.poses[i].pose.position.z - path.poses[i-1].pose.position.z);
  }

  double total_energy = 0.0;
  for (int i=1; i < vel_x.size(); ++i) {
    total_energy += std::abs(vel_x[i]*vel_x[i] - vel_x[i-1]*vel_x[i-1]);
    total_energy += std::abs(vel_y[i]*vel_y[i] - vel_y[i-1]*vel_y[i-1]);
    total_energy += std::abs(vel_z[i]*vel_z[i] - vel_z[i-1]*vel_z[i-1]);
  }
  return total_energy;
}

double pathEnergy(nav_msgs::Path & path, double up_penalty) {
  double total_energy = 0.0;
  for (int i=1; i < path.poses.size(); ++i) {
    total_energy += distance(path.poses[i-1], path.poses[i]);
    double altitude_increase = path.poses[i].pose.position.z - path.poses[i-1].pose.position.z;
    total_energy += std::max(0.0, up_penalty * altitude_increase);
  }
  return total_energy;
}

} // namespace avoidance

#endif /* GLOBAL_PLANNER_COMMON_H_ */
