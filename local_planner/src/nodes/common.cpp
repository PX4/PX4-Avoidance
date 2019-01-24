#include "common.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <vector>

#include <tf/transform_listener.h>
namespace avoidance {

float distance2DPolar(const PolarPoint& p1, const PolarPoint& p2) {
  return sqrt((p1.e - p2.e) * (p1.e - p2.e) + (p1.z - p2.z) * (p1.z - p2.z));
}

// transform polar coordinates into Cartesian coordinates

Eigen::Vector3f PolarToCartesian(const PolarPoint& p_pol,
                                 const geometry_msgs::Point& pos) {
  Eigen::Vector3f p;
  p.x() =
      pos.x +
      p_pol.r * cos(p_pol.e * (M_PI / 180.f)) * sin(p_pol.z * (M_PI / 180.f));
  p.y() =
      pos.y +
      p_pol.r * cos(p_pol.e * (M_PI / 180.f)) * cos(p_pol.z * (M_PI / 180.f));
  p.z() = pos.z + p_pol.r * sin(p_pol.e * (M_PI / 180.f));

  return p;
}
double indexAngleDifference(float a, float b) {
  return std::min(std::min(std::abs(a - b), std::abs(a - b - 360.f)),
                  std::abs(a - b + 360.f));
}

PolarPoint HistogramIndexToPolar(int e, int z, int res, const float& radius) {
  PolarPoint p_pol = {};
  p_pol.e = e * res + res / 2 - 90;
  p_pol.z = z * res + res / 2 - 180;
  p_pol.r = radius;
  return p_pol;
}

PolarPoint CartesianToPolar(const Eigen::Vector3f& pos,
                            const Eigen::Vector3f& origin) {
  return CartesianToPolar(pos.x(), pos.y(), pos.z(), origin);
}
PolarPoint CartesianToPolar(double x, double y, double z,
                            const Eigen::Vector3f& pos) {
  PolarPoint p_pol = {};
  p_pol.z = atan2(x - pos.x(), y - pos.y()) * (180.0 / M_PI);  //(-180. +180]

  double den = (Eigen::Vector2f(x, y) - pos.topRows<2>()).norm();
  p_pol.e = atan2(z - pos.z(), den) * 180.0 / M_PI;  //(-90.+90)
  p_pol.r = sqrt((x - pos.x()) * (x - pos.x()) + (y - pos.y()) * (y - pos.y()) +
                 (z - pos.z()) * (z - pos.z()));
  return p_pol;
}

Eigen::Vector2i PolarToHistogramIndex(const PolarPoint& p_pol, int res) {
  // TODO change logic here, as 0,0 are valid index
  Eigen::Vector2i ev2(0, 0);
  float e = p_pol.e;
  float z = p_pol.z;
  if (res <= 0.f || e < -90.f || e > 90.f || z < -180.f || z > 180.f) {
    return ev2;
  }
  if (e == 90.f) {
    e = 89;
  }
  e += 90.0;
  e = e + (res - (static_cast<int>(e) % res));  //[-80,+90]
  ev2.y() = e / res - 1;

  if (z == 180.f) {
    z = -180;
  }
  z += 180.0;
  z = z + (res - (static_cast<int>(z) % res));  //[-80,+90]
  ev2.x() = z / res - 1;
  return ev2;
}

// calculate the yaw for the next waypoint
double nextYaw(const geometry_msgs::PoseStamped& u,
               const geometry_msgs::Point& v) {
  double dx = v.x - u.pose.position.x;
  double dy = v.y - u.pose.position.y;

  return atan2(dy, dx);
}

geometry_msgs::PoseStamped createPoseMsg(const geometry_msgs::Point& waypt,
                                         double yaw) {
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.header.frame_id = "/local_origin";
  pose_msg.pose.position.x = waypt.x;
  pose_msg.pose.position.y = waypt.y;
  pose_msg.pose.position.z = waypt.z;
  pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
  return pose_msg;
}

double velocityLinear(double max_vel, double slope, double v_old,
                      double elapsed) {
  double t_old = v_old / slope;
  double t_new = t_old + elapsed;
  double speed = t_new * slope;
  if (speed > max_vel) {
    speed = max_vel;
  }
  return speed;
}

void wrapAngleToPlusMinusPI(double& angle) {
  while (angle > M_PI) {
    angle -= 2 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2 * M_PI;
  }
}

double getAngularVelocity(double desired_yaw, double curr_yaw) {
  wrapAngleToPlusMinusPI(desired_yaw);
  double yaw_vel1 = desired_yaw - curr_yaw;
  double yaw_vel2;
  // finds the yaw vel for the other yaw direction
  if (yaw_vel1 > 0) {
    yaw_vel2 = -(2 * M_PI - yaw_vel1);
  } else {
    yaw_vel2 = 2 * M_PI + yaw_vel1;
  }

  // check which yaw direction is shorter
  double vel = (std::abs(yaw_vel1) <= std::abs(yaw_vel2)) ? yaw_vel1 : yaw_vel2;
  return 0.5 * vel;
}

Eigen::Vector3f toEigen(const geometry_msgs::Point& p) {
  Eigen::Vector3f ev3(p.x, p.y, p.z);
  return ev3;
}
Eigen::Vector3f toEigen(const pcl::PointXYZ& p) {
  Eigen::Vector3f ev3(p.x, p.y, p.z);
  return ev3;
}

geometry_msgs::Point toPoint(const Eigen::Vector3f& ev3) {
  geometry_msgs::Point gmp;
  gmp.x = ev3.x();
  gmp.y = ev3.y();
  gmp.z = ev3.z();
  return gmp;
}
pcl::PointXYZ toXYZ(const Eigen::Vector3f& ev3) {
  pcl::PointXYZ xyz;
  xyz.x = ev3.x();
  xyz.y = ev3.y();
  xyz.z = ev3.z();
  return xyz;
}
}
