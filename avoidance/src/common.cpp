#include "avoidance/common.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <vector>

namespace avoidance {

bool pointInsideFOV(const FOV& fov, const PolarPoint& p_pol) {
  // pitch angle must be counted negative: zero pitch is level, positive pitch
  // is leaning down. Whereas a positive elevation angle is counted upwards
  return p_pol.z <= fov.yaw_deg + fov.h_fov_deg / 2.f &&
         p_pol.z >= fov.yaw_deg - fov.h_fov_deg / 2.f &&
         p_pol.e <= -fov.pitch_deg + fov.v_fov_deg / 2.f &&
         p_pol.e >= -fov.pitch_deg - fov.v_fov_deg / 2.f;
}

float distance2DPolar(const PolarPoint& p1, const PolarPoint& p2) {
  return sqrt((p1.e - p2.e) * (p1.e - p2.e) + (p1.z - p2.z) * (p1.z - p2.z));
}

Eigen::Vector3f polarToCartesian(const PolarPoint& p_pol,
                                 const Eigen::Vector3f& pos) {
  Eigen::Vector3f p;
  p.x() =
      pos.x() +
      p_pol.r * std::cos(p_pol.e * DEG_TO_RAD) * std::sin(p_pol.z * DEG_TO_RAD);
  p.y() =
      pos.y() +
      p_pol.r * std::cos(p_pol.e * DEG_TO_RAD) * std::cos(p_pol.z * DEG_TO_RAD);
  p.z() = pos.z() + p_pol.r * std::sin(p_pol.e * DEG_TO_RAD);

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
PolarPoint cartesianToPolar(float x, float y, float z,
                            const Eigen::Vector3f& pos) {
  PolarPoint p_pol(0.0f, 0.0f, 0.0f);
  float den = (Eigen::Vector2f(x, y) - pos.topRows<2>()).norm();
  p_pol.e = std::atan2(z - pos.z(), den) * RAD_TO_DEG;          //(-90.+90)
  p_pol.z = std::atan2(x - pos.x(), y - pos.y()) * RAD_TO_DEG;  //(-180. +180]
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
float nextYaw(const Eigen::Vector3f& u, const Eigen::Vector3f& v) {
  float dx = v.x() - u.x();
  float dy = v.y() - u.y();

  return atan2(dy, dx);
}

void createPoseMsg(Eigen::Vector3f& out_waypt, Eigen::Quaternionf& out_q,
                   const Eigen::Vector3f& in_waypt, float yaw) {
  out_waypt = in_waypt;
  float roll = 0.0f, pitch = 0.0f;
  out_q = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()) *
          Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
          Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
}

float getYawFromQuaternion(const Eigen::Quaternionf q) {
  float siny_cosp = 2.f * (q.w() * q.z() + q.x() * q.y());
  float cosy_cosp = 1.f - 2.f * (q.y() * q.y() + q.z() * q.z());
  float yaw = atan2(siny_cosp, cosy_cosp);

  return yaw * RAD_TO_DEG;
}

float getPitchFromQuaternion(const Eigen::Quaternionf q) {
  float pitch = 0.f;
  float sinp = 2.f * (q.w() * q.y() - q.z() * q.x());
  if (fabs(sinp) >= 1.f) {
    pitch = copysign(M_PI / 2.f, sinp);  // use PI/2 if out of range
  } else {
    pitch = asin(sinp);
  }
  return pitch * RAD_TO_DEG;
}

void wrapAngleToPlusMinusPI(float& angle) {
  angle = angle - 2.0f * M_PI_F * std::floor(angle / (2.0f * M_PI_F) + 0.5f);
}

void wrapAngleToPlusMinus180(float& angle) {
  angle = angle - 360.f * std::floor(angle / 360.f + 0.5f);
}

double getAngularVelocity(float desired_yaw, float curr_yaw) {
  wrapAngleToPlusMinusPI(desired_yaw);
  float yaw_vel1 = desired_yaw - curr_yaw;
  float yaw_vel2;
  // finds the yaw vel for the other yaw direction
  if (yaw_vel1 > 0.0f) {
    yaw_vel2 = -(2.0f * M_PI_F - yaw_vel1);
  } else {
    yaw_vel2 = 2.0f * M_PI_F + yaw_vel1;
  }

  // check which yaw direction is shorter
  float vel = (std::abs(yaw_vel1) <= std::abs(yaw_vel2)) ? yaw_vel1 : yaw_vel2;
  return 0.5 * static_cast<double>(vel);
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

Eigen::Vector3f toEigen(const pcl::PointXYZI& p) {
  Eigen::Vector3f ev3(p.x, p.y, p.z);
  return ev3;
}

Eigen::Quaternionf toEigen(const geometry_msgs::Quaternion& gmq) {
  Eigen::Quaternionf eqf;
  eqf.x() = gmq.x;
  eqf.y() = gmq.y;
  eqf.z() = gmq.z;
  eqf.w() = gmq.w;
  return eqf;
}

geometry_msgs::Point toPoint(const Eigen::Vector3f& ev3) {
  geometry_msgs::Point gmp;
  gmp.x = ev3.x();
  gmp.y = ev3.y();
  gmp.z = ev3.z();
  return gmp;
}

geometry_msgs::Vector3 toVector3(const Eigen::Vector3f& ev3) {
  geometry_msgs::Vector3 gmv3;
  gmv3.x = ev3.x();
  gmv3.y = ev3.y();
  gmv3.z = ev3.z();
  return gmv3;
}

geometry_msgs::Quaternion toQuaternion(const Eigen::Quaternionf& eqf) {
  geometry_msgs::Quaternion q;
  q.x = eqf.x();
  q.y = eqf.y();
  q.z = eqf.z();
  q.w = eqf.w();
  return q;
}

pcl::PointXYZ toXYZ(const Eigen::Vector3f& ev3) {
  pcl::PointXYZ xyz;
  xyz.x = ev3.x();
  xyz.y = ev3.y();
  xyz.z = ev3.z();
  return xyz;
}

pcl::PointXYZI toXYZI(const Eigen::Vector3f& ev3, float intensity) {
  pcl::PointXYZI p;
  p.x = ev3.x();
  p.y = ev3.y();
  p.z = ev3.z();
  p.intensity = intensity;
  return p;
}

pcl::PointXYZI toXYZI(float x, float y, float z, float intensity) {
  pcl::PointXYZI p;
  p.x = x;
  p.y = y;
  p.z = z;
  p.intensity = intensity;
  return p;
}

pcl::PointXYZI toXYZI(const pcl::PointXYZ& xyz, float intensity) {
  pcl::PointXYZI p;
  p.x = xyz.x;
  p.y = xyz.y;
  p.z = xyz.z;
  p.intensity = intensity;
  return p;
}

geometry_msgs::Twist toTwist(const Eigen::Vector3f& l,
                             const Eigen::Vector3f& a) {
  geometry_msgs::Twist gmt;
  gmt.linear = toVector3(l);
  gmt.angular = toVector3(a);
  return gmt;
}

geometry_msgs::PoseStamped toPoseStamped(const Eigen::Vector3f& ev3,
                                         const Eigen::Quaternionf& eq) {
  geometry_msgs::PoseStamped gmps;
  gmps.header.stamp = ros::Time::now();
  gmps.header.frame_id = "/local_origin";
  gmps.pose.position = toPoint(ev3);
  gmps.pose.orientation = toQuaternion(eq);
  return gmps;
}

void transformPoseToTrajectory(mavros_msgs::Trajectory& obst_avoid,
                               geometry_msgs::PoseStamped pose) {
  obst_avoid.header = pose.header;
  obst_avoid.type = 0;  // MAV_TRAJECTORY_REPRESENTATION::WAYPOINTS
  obst_avoid.point_1.position.x = pose.pose.position.x;
  obst_avoid.point_1.position.y = pose.pose.position.y;
  obst_avoid.point_1.position.z = pose.pose.position.z;
  obst_avoid.point_1.velocity.x = NAN;
  obst_avoid.point_1.velocity.y = NAN;
  obst_avoid.point_1.velocity.z = NAN;
  obst_avoid.point_1.acceleration_or_force.x = NAN;
  obst_avoid.point_1.acceleration_or_force.y = NAN;
  obst_avoid.point_1.acceleration_or_force.z = NAN;
  obst_avoid.point_1.yaw = tf::getYaw(pose.pose.orientation);
  obst_avoid.point_1.yaw_rate = NAN;

  fillUnusedTrajectoryPoint(obst_avoid.point_2);
  fillUnusedTrajectoryPoint(obst_avoid.point_3);
  fillUnusedTrajectoryPoint(obst_avoid.point_4);
  fillUnusedTrajectoryPoint(obst_avoid.point_5);

  obst_avoid.time_horizon = {NAN, NAN, NAN, NAN, NAN};

  obst_avoid.point_valid = {true, false, false, false, false};
}

void transformVelocityToTrajectory(mavros_msgs::Trajectory& obst_avoid,
                                   geometry_msgs::Twist vel) {
  obst_avoid.header.stamp = ros::Time::now();
  obst_avoid.type = 0;  // MAV_TRAJECTORY_REPRESENTATION::WAYPOINTS
  obst_avoid.point_1.position.x = NAN;
  obst_avoid.point_1.position.y = NAN;
  obst_avoid.point_1.position.z = NAN;
  obst_avoid.point_1.velocity.x = vel.linear.x;
  obst_avoid.point_1.velocity.y = vel.linear.y;
  obst_avoid.point_1.velocity.z = vel.linear.z;
  obst_avoid.point_1.acceleration_or_force.x = NAN;
  obst_avoid.point_1.acceleration_or_force.y = NAN;
  obst_avoid.point_1.acceleration_or_force.z = NAN;
  obst_avoid.point_1.yaw = NAN;
  obst_avoid.point_1.yaw_rate = -vel.angular.z;

  fillUnusedTrajectoryPoint(obst_avoid.point_2);
  fillUnusedTrajectoryPoint(obst_avoid.point_3);
  fillUnusedTrajectoryPoint(obst_avoid.point_4);
  fillUnusedTrajectoryPoint(obst_avoid.point_5);

  obst_avoid.time_horizon = {NAN, NAN, NAN, NAN, NAN};

  obst_avoid.point_valid = {true, false, false, false, false};
}

void fillUnusedTrajectoryPoint(mavros_msgs::PositionTarget& point) {
  point.position.x = NAN;
  point.position.y = NAN;
  point.position.z = NAN;
  point.velocity.x = NAN;
  point.velocity.y = NAN;
  point.velocity.z = NAN;
  point.acceleration_or_force.x = NAN;
  point.acceleration_or_force.y = NAN;
  point.acceleration_or_force.z = NAN;
  point.yaw = NAN;
  point.yaw_rate = NAN;
}
}
