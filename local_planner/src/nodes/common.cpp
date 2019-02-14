#include "local_planner/common.h"

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

Eigen::Vector3f polarToCartesian(const PolarPoint& p_pol,
                                 const geometry_msgs::Point& pos) {
  Eigen::Vector3f p;
  p.x() =
      pos.x + p_pol.r * cos(p_pol.e * DEG_TO_RAD) * sin(p_pol.z * DEG_TO_RAD);
  p.y() =
      pos.y + p_pol.r * cos(p_pol.e * DEG_TO_RAD) * cos(p_pol.z * DEG_TO_RAD);
  p.z() = pos.z + p_pol.r * sin(p_pol.e * DEG_TO_RAD);

  return p;
}
float indexAngleDifference(float a, float b) {
  return std::min(std::min(std::abs(a - b), std::abs(a - b - 360.f)),
                  std::abs(a - b + 360.f));
}

PolarPoint histogramIndexToPolar(int e, int z, int res, float radius) {
  // ALPHA_RES%2=0 as per definition, see histogram.h
  PolarPoint p_pol(static_cast<float>(e * res + res / 2 - 90),
                   static_cast<float>(z * res + res / 2 - 180), radius);
  return p_pol;
}

PolarPoint cartesianToPolar(const Eigen::Vector3f& pos,
                            const Eigen::Vector3f& origin) {
  return cartesianToPolar(pos.x(), pos.y(), pos.z(), origin);
}
PolarPoint cartesianToPolar(double x, double y, double z,
                            const Eigen::Vector3f& pos) {
  PolarPoint p_pol(0.0f, 0.0f, 0.0f);
  double den = (Eigen::Vector2f(x, y) - pos.topRows<2>()).norm();
  p_pol.e = atan2(z - pos.z(), den) * 180.0 / M_PI;            //(-90.+90)
  p_pol.z = atan2(x - pos.x(), y - pos.y()) * (180.0 / M_PI);  //(-180. +180]
  p_pol.r = sqrt((x - pos.x()) * (x - pos.x()) + (y - pos.y()) * (y - pos.y()) +
                 (z - pos.z()) * (z - pos.z()));
  return p_pol;
}

Eigen::Vector2i polarToHistogramIndex(const PolarPoint& p_pol, int res) {
  Eigen::Vector2i ev2(0, 0);
  PolarPoint p_wrapped = p_pol;
  wrapPolar(p_wrapped);
  // elevation angle to y-axis histogram index
  // maps elevation -90° to bin 0 and +90° to the highest bin (N-1)
  ev2.y() = static_cast<int>(floor(p_wrapped.e / res + 90.0f / res));
  // azimuth angle to x-axis histogram index
  // maps elevation -180° to bin 0 and +180° to the highest bin (N-1)
  ev2.x() = static_cast<int>(floor(p_wrapped.z / res + 180.0f / res));

  // clamp due to floating point errros
  if (ev2.x() >= 360 / res) ev2.x() = 360 / res - 1;
  if (ev2.x() < 0) ev2.x() = 0;
  if (ev2.y() >= 180 / res) ev2.y() = 180 / res - 1;
  if (ev2.y() < 0) ev2.y() = 0;

  return ev2;
}

void wrapPolar(PolarPoint& p_pol) {
  // first wrap the angles to +-180 degrees
  wrapAngleToPlusMinus180(p_pol.e);
  wrapAngleToPlusMinus180(p_pol.z);

  // elevation valid [-90,90)
  // when abs(elevation) > 90, wrap elevation angle
  // azimuth changes 180 if it wraps

  bool wrapped = false;
  if (p_pol.e > 90.0f) {
    p_pol.e = 180.0f - p_pol.e;
    wrapped = true;
  } else if (p_pol.e < -90.0f) {
    p_pol.e = -(180.0f + p_pol.e);
    wrapped = true;
  }
  if (wrapped) {
    if (p_pol.z < 0.f)
      p_pol.z += 180.f;
    else
      p_pol.z -= 180.f;
  }
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
  angle = angle - 2 * M_PI * floor(angle / (2 * M_PI) + 0.5);
}

void wrapAngleToPlusMinus180(float& angle) {
  angle = angle - 360.f * floor(angle / 360.f + 0.5f);
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

Eigen::Vector3f toEigen(const geometry_msgs::Vector3& v3) {
  Eigen::Vector3f ev3(v3.x, v3.y, v3.z);
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
