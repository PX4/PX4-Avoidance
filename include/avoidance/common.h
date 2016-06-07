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

#include <glog/logging.h>

#include <nav_msgs/Path.h>
 

namespace avoidance {

// GLOBAL PLANNER

template <typename T>
double squared(T x) {
  return x * x;
}

double distance(geometry_msgs::PoseStamped & a, geometry_msgs::PoseStamped & b) {
  double diffX = a.pose.position.x - b.pose.position.x;
  double diffY = a.pose.position.y - b.pose.position.y;
  double diffZ = a.pose.position.z - b.pose.position.z;
  return sqrt(diffX*diffX + diffY*diffY + diffZ*diffZ);
}

double interpolate(double start, double end, double ratio) {
  return start + (end - start) * ratio;
}

double angleToRange(double angle) {
  // returns the angle in the range [-pi, pi]
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

bool hasSameYawAndAltitude(const geometry_msgs::Pose& msg1,
                         const geometry_msgs::Pose& msg2) {

  return msg1.orientation.z == msg2.orientation.z
      && msg1.orientation.w == msg2.orientation.w
      && msg1.position.z == msg2.position.z;
}

double posterior(double p, double prior) {
  // p and prior are independent measurements of the same event
  double isObst = p * prior;
  double isNotObst = (1-p) * (1-prior);
  return isObst / (isObst + isNotObst+0.0001);
}

double pathLength(nav_msgs::Path & path) {
  // p and prior are independent measurements of the same event
  double totalDist = 0.0;
  for (int i=1; i < path.poses.size(); ++i) {
    totalDist += distance(path.poses[i-1], path.poses[i]);
  }
  return totalDist;
}

double pathKineticEnergy(nav_msgs::Path & path) {
  // p and prior are independent measurements of the same event
  if (path.poses.size() < 3) {
    return 0.0;
  }
  std::vector<double> velX;
  std::vector<double> velY;
  std::vector<double> velZ;
  for (int i=1; i < path.poses.size(); ++i) {
    velX.push_back(path.poses[i].pose.position.x - path.poses[i-1].pose.position.x);
    velY.push_back(path.poses[i].pose.position.y - path.poses[i-1].pose.position.y);
    velZ.push_back(path.poses[i].pose.position.z - path.poses[i-1].pose.position.z);
  }

  double totalEnergy = 0.0;
  for (int i=1; i < velX.size(); ++i) {
    totalEnergy += std::abs(velX[i]*velX[i] - velX[i-1]*velX[i-1]);
    totalEnergy += std::abs(velY[i]*velY[i] - velY[i-1]*velY[i-1]);
    totalEnergy += std::abs(velZ[i]*velZ[i] - velZ[i-1]*velZ[i-1]);
  }

  return totalEnergy;
}

double pathEnergy(nav_msgs::Path & path, double upPenalty) {
  // p and prior are independent measurements of the same event
  double totalEnergy = 0.0;
  for (int i=1; i < path.poses.size(); ++i) {
    totalEnergy += distance(path.poses[i-1], path.poses[i]);
    double altitudeIncrease = path.poses[i].pose.position.z - path.poses[i-1].pose.position.z;
    totalEnergy += std::max(0.0, upPenalty * altitudeIncrease);
  }
  return totalEnergy;
}

} // namespace avoidance

#endif /* GLOBAL_PLANNER_COMMON_H_ */
