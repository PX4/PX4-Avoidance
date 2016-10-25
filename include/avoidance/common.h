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
tf::Vector3 toTfVector3(const P & point) {
  return tf::Vector3(point.x, point.y, point.z);
}

template <typename P>
void setPointCoordinates(P & point, double x, double y, double z) {
  point.x = x;
  point.y = y;
  point.z = z;
}

// Returns the point in the middle of the line segment between p1 and p2
template <typename P>
P middlePoint(const P & p1, const P & p2) {
  P new_point;
  new_point.x = (p1.x + p2.x) / 2.0;
  new_point.y = (p1.y + p2.y) / 2.0;
  new_point.z = (p1.z + p2.z) / 2.0;
  return new_point;
}

template <typename P>
P addPoints(const P & p1, const P & p2) {
  P new_p;
  new_p.x = p1.x + p2.x;
  new_p.y = p1.y + p2.y;
  new_p.z = p1.z + p2.z;
  return new_p;
}

template <typename P>
P subtractPoints(const P & p1, const P & p2) {
  P new_p;
  new_p.x = p1.x - p2.x;
  new_p.y = p1.y - p2.y;
  new_p.z = p1.z - p2.z;
  return new_p;
}

template <typename P>
double distance(const P & p1, const P & p2) {
  return squared(p2.x - p1.x) + squared(p2.y - p1.y) + squared(p2.z - p1.z);
}

double distance(const geometry_msgs::PoseStamped & a, const geometry_msgs::PoseStamped & b) {
  return distance(a.pose.position, b.pose.position);
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

double norm(const geometry_msgs::Vector3 & v) {
  return std::sqrt(squared(v.x) + squared(v.y) + squared(v.z));
}

// Returns a weighted average of start and end, where ratio is the weight of start 
double interpolate(double start, double end, double ratio) {
  return start + (end - start) * ratio;
}

// Returns the point on the quadratic Bezier curve at time t (0 <= t <= 1)
template <typename T>
T quadraticBezier(T p0, T p1, T p2, double t) {
  return ((1 - t) * (1 - t) * p0) + 2 * ((1 - t) * t * p1) + (t * t * p2);
}

// Returns a quadratic Bezier-curve starting in p0 and and ending in p2
template <typename P>
std::vector<P> threePointBezier(const P & p0, const P & p1, const P & p2, int num_steps = 10) {
  std::vector<P> curve;
  for (int i=0; i <= num_steps; ++i) {
    double t = ((double) i) / num_steps;
    P new_point;
    new_point.x = quadraticBezier(p0.x, p1.x, p2.x, t);
    new_point.y = quadraticBezier(p0.y, p1.y, p2.y, t);
    new_point.z = quadraticBezier(p0.z, p1.z, p2.z, t);
    curve.push_back(new_point);
  }
  return curve;
}

// Returns a path where corners are smoothed with quadratic Bezier-curves
nav_msgs::Path smoothPath(const nav_msgs::Path & path) {
  if (path.poses.size() < 3) {
    return path;
  }
  
  // Repeat the first and last points to get the first half of the first edge 
  // and the second half of the last edge
  std::vector<geometry_msgs::PoseStamped> corner_points = path.poses;
  corner_points.insert(corner_points.begin(), corner_points.front());
  corner_points.push_back(corner_points.back());

  nav_msgs::Path smooth_path;
  smooth_path.header = path.header;

  for (int i=2; i < path.poses.size(); i++) {
    geometry_msgs::Point p0 = path.poses[i-2].pose.position;
    geometry_msgs::Point p1 = path.poses[i-1].pose.position;
    geometry_msgs::Point p2 = path.poses[i].pose.position;
    p0 = middlePoint(p0, p1);
    p2 = middlePoint(p1, p2);

    std::vector<geometry_msgs::Point> smooth_turn = threePointBezier(p0, p1, p2);
    for (const auto & point : smooth_turn) {
      geometry_msgs::PoseStamped pose_msg = path.poses.front(); // Copy the original header info
      pose_msg.pose.position = point;
      smooth_path.poses.push_back(pose_msg);
    }
  }
  return smooth_path;
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

double pathLength(const nav_msgs::Path & path) {
  double total_dist = 0.0;
  for (int i=1; i < path.poses.size(); ++i) {
    total_dist += distance(path.poses[i-1], path.poses[i]);
  }
  return total_dist;
}

// Returns a path with only the corner points of msg
std::vector<geometry_msgs::PoseStamped> filterPathCorners(const std::vector<geometry_msgs::PoseStamped> & msg) {
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


double pathKineticEnergy(const nav_msgs::Path & path) {
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

double pathEnergy(const nav_msgs::Path & path, double up_penalty) {
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
