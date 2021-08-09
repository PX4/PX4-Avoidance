#include "avoidance/common.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <vector>

namespace avoidance {

bool pointInsideFOV(const std::vector<FOV>& fov_vec, const PolarPoint& p_pol) {
  for (auto fov : fov_vec) {
    if (pointInsideFOV(fov, p_pol)) {
      return true;
    }
  }
  return false;
}

bool pointInsideFOV(const FOV& fov, const PolarPoint& p_pol) {
  return p_pol.z <= wrapAngleToPlusMinus180(fov.yaw_deg + fov.h_fov_deg / 2.f) &&
         p_pol.z >= wrapAngleToPlusMinus180(fov.yaw_deg - fov.h_fov_deg / 2.f) &&
         p_pol.e <= fov.pitch_deg + fov.v_fov_deg / 2.f && p_pol.e >= fov.pitch_deg - fov.v_fov_deg / 2.f;
}

bool histogramIndexYawInsideFOV(const std::vector<FOV>& fov_vec, const int idx, Eigen::Vector3f position,
                                float yaw_fcu_frame) {
  PolarPoint pol_hist = histogramIndexToPolar(GRID_LENGTH_E / 2, idx, ALPHA_RES, 1.f);
  Eigen::Vector3f cart = polarHistogramToCartesian(pol_hist, position);
  PolarPoint pol_fcu = cartesianToPolarFCU(cart, position);  // z down convention
  pol_fcu.z -= yaw_fcu_frame;                                // transform to fcu body frame
  PolarPoint pol_fcu_plus = pol_fcu;
  PolarPoint pol_fcu_minus = pol_fcu;
  pol_fcu_plus.z += ALPHA_RES / 2.f;
  pol_fcu_minus.z -= ALPHA_RES / 2.f;
  wrapPolar(pol_fcu_plus);
  wrapPolar(pol_fcu_minus);

  return (pointInsideFOV(fov_vec, pol_fcu_plus) || pointInsideFOV(fov_vec, pol_fcu_minus));
}

bool histogramIndexYawInsideFOV(const FOV& fov, const int idx, Eigen::Vector3f position, float yaw_fcu_frame) {
  PolarPoint pol_hist = histogramIndexToPolar(GRID_LENGTH_E / 2, idx, ALPHA_RES, 1.f);
  Eigen::Vector3f cart = polarHistogramToCartesian(pol_hist, position);
  PolarPoint pol_fcu = cartesianToPolarFCU(cart, position);  // z down convention
  pol_fcu.z -= yaw_fcu_frame;                                // transform to fcu body frame
  PolarPoint pol_fcu_plus = pol_fcu;
  PolarPoint pol_fcu_minus = pol_fcu;
  pol_fcu_plus.z += ALPHA_RES / 2.f;
  pol_fcu_minus.z -= ALPHA_RES / 2.f;
  wrapPolar(pol_fcu_plus);
  wrapPolar(pol_fcu_minus);

  return (pointInsideFOV(fov, pol_fcu_plus) || pointInsideFOV(fov, pol_fcu_minus));
}

bool pointInsideYawFOV(const std::vector<FOV>& fov_vec, const PolarPoint& p_pol) {
  for (auto fov : fov_vec) {
    if (pointInsideYawFOV(fov, p_pol)) {
      return true;
    }
  }
  return false;
}

bool pointInsideYawFOV(const FOV& fov, const PolarPoint& p_pol) {
  return p_pol.z <= wrapAngleToPlusMinus180(fov.yaw_deg + fov.h_fov_deg / 2.f) &&
         p_pol.z >= wrapAngleToPlusMinus180(fov.yaw_deg - fov.h_fov_deg / 2.f);
}

bool isInWhichFOV(const std::vector<FOV>& fov_vec, const PolarPoint& p_pol, int& idx) {
  bool retval = false;
  idx = -1;
  for (size_t i = 0; i < fov_vec.size(); ++i) {
    if (pointInsideYawFOV(fov_vec[i], p_pol)) {
      if (retval) {  // if it's been found before, return false!
        idx = -1;
        return false;
      }
      idx = i;
      retval = true;
    }
  }
  return retval;
}

bool isOnEdgeOfFOV(const std::vector<FOV>& fov_vec, const PolarPoint& p_pol, int& idx) {
  idx = -1;
  bool retval = false;
  if (isInWhichFOV(fov_vec, p_pol, idx)) {
    PolarPoint just_outside = p_pol;
    // todo: check for pitch!
    if (wrapAngleToPlusMinus180(p_pol.z - fov_vec[idx].yaw_deg) > 0.0f) {
      // check half-resolution degrees outside the current fov
      just_outside.z =
          wrapAngleToPlusMinus180(fov_vec[idx].yaw_deg + fov_vec[idx].h_fov_deg / 2.0f + (ALPHA_RES / 2.0f));
    } else {  // half-resolution degrees to the left
      just_outside.z =
          wrapAngleToPlusMinus180(fov_vec[idx].yaw_deg - fov_vec[idx].h_fov_deg / 2.0f - (ALPHA_RES / 2.0f));
    }

    retval = !pointInsideYawFOV(fov_vec, just_outside);
    if (!retval) {
      idx = -1;
    }
  }
  return retval;
}

float scaleToFOV(const std::vector<FOV>& fov, const PolarPoint& p_pol) {
  int i;
  if (isOnEdgeOfFOV(fov, p_pol, i)) {
    float angle_diff_deg = std::abs(fov[i].yaw_deg - p_pol.z);
    angle_diff_deg = std::min(angle_diff_deg, std::abs(360.f - angle_diff_deg));
    angle_diff_deg = std::min(fov[i].h_fov_deg / 2.0f, angle_diff_deg);  // Clamp at h_FOV/2
    return 1.0f - 2.0f * angle_diff_deg / fov[i].h_fov_deg;
  }
  return pointInsideYawFOV(fov, p_pol) ? 1.f : 0.f;
}

float distance2DPolar(const PolarPoint& p1, const PolarPoint& p2) {
  return sqrt((p1.e - p2.e) * (p1.e - p2.e) + (p1.z - p2.z) * (p1.z - p2.z));
}

Eigen::Vector3f polarHistogramToCartesian(const PolarPoint& p_pol, const Eigen::Vector3f& pos) {
  Eigen::Vector3f p;
  p.x() = pos.x() + p_pol.r * std::cos(p_pol.e * DEG_TO_RAD) * std::sin(p_pol.z * DEG_TO_RAD);
  p.y() = pos.y() + p_pol.r * std::cos(p_pol.e * DEG_TO_RAD) * std::cos(p_pol.z * DEG_TO_RAD);
  p.z() = pos.z() + p_pol.r * std::sin(p_pol.e * DEG_TO_RAD);

  return p;
}

Eigen::Vector3f polarFCUToCartesian(const PolarPoint& p_pol, const Eigen::Vector3f& pos) {
  Eigen::Vector3f p;
  p.x() = pos.x() + p_pol.r * std::sin((90.0f - p_pol.e) * DEG_TO_RAD) * std::cos(p_pol.z * DEG_TO_RAD);
  p.y() = pos.y() + p_pol.r * std::sin((90.0f - p_pol.e) * DEG_TO_RAD) * std::sin(p_pol.z * DEG_TO_RAD);
  p.z() = pos.z() + p_pol.r * std::cos((90.0f - p_pol.e) * DEG_TO_RAD);

  return p;
}

float indexAngleDifference(float a, float b) {
  return std::min(std::min(std::abs(a - b), std::abs(a - b - 360.f)), std::abs(a - b + 360.f));
}

PolarPoint histogramIndexToPolar(int e, int z, int res, float radius) {
  // ALPHA_RES%2=0 as per definition, see histogram.h
  PolarPoint p_pol(static_cast<float>(e * res + res / 2 - 90), static_cast<float>(z * res + res / 2 - 180), radius);
  return p_pol;
}

PolarPoint cartesianToPolarHistogram(const Eigen::Vector3f& pos, const Eigen::Vector3f& origin) {
  return cartesianToPolarHistogram(pos.x(), pos.y(), pos.z(), origin);
}
PolarPoint cartesianToPolarHistogram(float x, float y, float z, const Eigen::Vector3f& pos) {
  PolarPoint p_pol(0.0f, 0.0f, 0.0f);
  float den = (Eigen::Vector2f(x, y) - pos.topRows<2>()).norm();
  p_pol.e = std::atan2(z - pos.z(), den) * RAD_TO_DEG;          //(-90.+90)
  p_pol.z = std::atan2(x - pos.x(), y - pos.y()) * RAD_TO_DEG;  //(-180. +180]
  p_pol.r = sqrt((x - pos.x()) * (x - pos.x()) + (y - pos.y()) * (y - pos.y()) + (z - pos.z()) * (z - pos.z()));
  return p_pol;
}

PolarPoint cartesianToPolarFCU(const Eigen::Vector3f& pos, const Eigen::Vector3f& origin) {
  PolarPoint p = cartesianToPolarHistogram(pos, origin);
  p.z = -p.z + 90.0f;
  p.e = -p.e;
  wrapPolar(p);
  return p;
}

PolarPoint cartesianToPolarFCU(const pcl::PointXYZ& p) {
  return cartesianToPolarFCU(Eigen::Vector3f(p.x, p.y, p.z), Eigen::Vector3f(0.0f, 0.0f, 0.0f));
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
  p_pol.e = wrapAngleToPlusMinus180(p_pol.e);
  p_pol.z = wrapAngleToPlusMinus180(p_pol.z);

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

void createPoseMsg(Eigen::Vector3f& out_waypt, Eigen::Quaternionf& out_q, const Eigen::Vector3f& in_waypt, float yaw) {
  out_waypt = in_waypt;
  float roll = 0.0f, pitch = 0.0f;
  out_q = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
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

geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw) {
  tf2::Quaternion quat_tf;
  quat_tf.setRPY( 0, 0, yaw );
  geometry_msgs::msg::Quaternion quat_msg;
  quat_msg = tf2::toMsg(quat_tf);
  return quat_msg;
}

geometry_msgs::msg::PoseStamped transformNEDandENU(geometry_msgs::msg::PoseStamped pose) {
  geometry_msgs::msg::TransformStamped transformStamped;
  tf2::Quaternion tf2_q_NED;
  tf2_q_NED.setRPY(3.14, 0, 1.57);
  geometry_msgs::msg::Quaternion geomsg_q;
  tf2::convert(tf2_q_NED, geomsg_q);
  transformStamped.transform.rotation = geomsg_q;
  
  geometry_msgs::msg::PoseStamped transformed_pose;
  tf2::doTransform(pose, transformed_pose, transformStamped);
  return transformed_pose;
}

float wrapAngleToPlusMinusPI(float angle) { return angle - 2.0f * M_PI_F * std::floor(angle / (2.0f * M_PI_F) + 0.5f); }

float wrapAngleToPlusMinus180(float angle) { return angle - 360.f * std::floor(angle / 360.f + 0.5f); }

float angleDifference(float a, float b) {
  float angle = fmod(a - b, 360.f);
  return angle >= 0.f ? (angle < 180.f) ? angle : angle - 360.f : (angle >= -180.f) ? angle : angle + 360.f;
}

double getAngularVelocity(float desired_yaw, float curr_yaw) {
  desired_yaw = wrapAngleToPlusMinusPI(desired_yaw);
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

void transformToTrajectory(px4_msgs::msg::VehicleTrajectoryWaypoint& obst_avoid, geometry_msgs::msg::PoseStamped pose,
                           geometry_msgs::msg::Twist vel) {

  obst_avoid.timestamp =  std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();

  obst_avoid.waypoints[0].position[0] = pose.pose.position.x;
  obst_avoid.waypoints[0].position[1] = pose.pose.position.y;
  obst_avoid.waypoints[0].position[2] = pose.pose.position.z;
  obst_avoid.waypoints[0].velocity[0] = vel.linear.x;
  obst_avoid.waypoints[0].velocity[1] = vel.linear.y;
  obst_avoid.waypoints[0].velocity[2] = vel.linear.z;
  obst_avoid.waypoints[0].acceleration[0] = NAN;
  obst_avoid.waypoints[0].acceleration[1] = NAN;
  obst_avoid.waypoints[0].acceleration[2] = NAN;
  obst_avoid.waypoints[0].yaw = tf2::getYaw(pose.pose.orientation);
  obst_avoid.waypoints[0].yaw_speed = -vel.angular.z;

  obst_avoid.type = 0;  // MAV_TRAJECTORY_REPRESENTATION::WAYPOINTS

  obst_avoid.waypoints[0].point_valid = true;

  fillUnusedTrajectoryPoint(obst_avoid.waypoints[1]);
  fillUnusedTrajectoryPoint(obst_avoid.waypoints[2]);
  fillUnusedTrajectoryPoint(obst_avoid.waypoints[3]);
  fillUnusedTrajectoryPoint(obst_avoid.waypoints[4]);

  for (size_t i = 0; i < obst_avoid.waypoints.size(); i++) {
    obst_avoid.waypoints[i].timestamp = obst_avoid.timestamp;
    obst_avoid.waypoints[i].point_valid = false;
  }
}

void fillUnusedTrajectoryPoint(px4_msgs::msg::TrajectoryWaypoint& point) {
  point.position[0] = NAN;
  point.position[1] = NAN;
  point.position[2] = NAN;
  point.velocity[0] = NAN;
  point.velocity[1] = NAN;
  point.velocity[2] = NAN;
  point.acceleration[0] = NAN;
  point.acceleration[1] = NAN;
  point.acceleration[2] = NAN;
  point.yaw = NAN;
  point.yaw_speed = NAN;
}

// This function is a refactor of the original in the pcl library
pcl::PointCloud<pcl::PointXYZ> removeNaNAndGetMaxima(pcl::PointCloud<pcl::PointXYZ>& cloud) {
  // Filter out NANs and keep track of outermost points for FOV
  size_t j = 0;
  float x_max = -9999.f, y_max = -9999.f, z_max = -9999.f, x_min = 9999.f, y_min = 9999.f, z_min = 9999.f;
  int i_x_max = -1, i_x_min = -1, i_y_max = -1, i_y_min = -1, i_z_max = -1, i_z_min = -1;
  for (size_t i = 0; i < cloud.points.size(); ++i) {
    if (!std::isfinite(cloud.points[i].x) || !std::isfinite(cloud.points[i].y) || !std::isfinite(cloud.points[i].z))
      continue;
    cloud.points[j] = cloud.points[i];  // safe, because i is always ahead of j

    if (cloud.points[j].x > x_max) {
      x_max = cloud.points[j].x;
      i_x_max = j;
    }

    if (cloud.points[j].y > y_max) {
      y_max = cloud.points[j].y;
      i_y_max = j;
    }

    if (cloud.points[j].z > z_max) {
      z_max = cloud.points[j].z;
      i_z_max = j;
    }

    if (cloud.points[j].x < x_min) {
      x_min = cloud.points[j].x;
      i_x_min = j;
    }

    if (cloud.points[j].y < y_min) {
      y_min = cloud.points[j].y;
      i_y_min = j;
    }

    if (cloud.points[j].z < z_min) {
      z_min = cloud.points[j].z;
      i_z_min = j;
    }

    j++;
  }
  if (j != cloud.points.size()) {
    // Resize to the correct size
    cloud.points.resize(j);
  }

  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(j);

  // Removing bad points => dense (note: 'dense' doesn't mean 'organized')
  cloud.is_dense = true;
  pcl::PointCloud<pcl::PointXYZ> maxima;
  maxima.header.frame_id = cloud.header.frame_id;
  if (i_x_max >= 0) maxima.push_back(cloud.points[i_x_max]);
  if (i_y_max >= 0) maxima.push_back(cloud.points[i_y_max]);
  if (i_z_max >= 0) maxima.push_back(cloud.points[i_z_max]);
  if (i_x_min >= 0) maxima.push_back(cloud.points[i_x_min]);
  if (i_y_min >= 0) maxima.push_back(cloud.points[i_y_min]);
  if (i_z_min >= 0) maxima.push_back(cloud.points[i_z_min]);

  return maxima;
}

void updateFOVFromMaxima(FOV& fov, const pcl::PointCloud<pcl::PointXYZ>& maxima) {
  float h_min = 9999.f, h_max = -9999.f, v_min = 9999.f, v_max = -9999.f;

  for (auto p : maxima) {
    PolarPoint p_pol_fcu = cartesianToPolarFCU(p);
    p_pol_fcu.z += 180.0f;  // move azimuth to [0, 360]
    p_pol_fcu.e += 90.0f;   // move elevation to [0, 180]
    h_min = std::min(p_pol_fcu.z, h_min);
    h_max = std::max(p_pol_fcu.z, h_max);
    v_min = std::min(p_pol_fcu.e, v_min);
    v_max = std::max(p_pol_fcu.e, v_max);
  }

  float h_diff = std::min(h_max - h_min, 360.0f - h_max + h_min);
  float v_diff = std::min(v_max - v_min, 360.0f - v_max + v_min);

  if (h_diff > fov.h_fov_deg) {
    fov.h_fov_deg = h_diff;

    // Note: the wrapping here assumes the FOV of one camera is < 180 degrees!
    if (h_diff >= h_max - h_min) {
      fov.yaw_deg = wrapAngleToPlusMinus180((h_max + h_min) / 2.0 - 180.0f);  // center of camera in [-180, 180]
    } else {
      fov.yaw_deg = wrapAngleToPlusMinus180((h_max + h_min) / 2.0);
    }
  }

  // Note: no wrapping in elevation! If the camera sees the zenith or nadir,
  // we are in trouble anyways! (aka. horizontal FOV = 360 degrees)
  if (v_diff > fov.v_fov_deg) {
    fov.v_fov_deg = v_diff;
    fov.pitch_deg = (v_max + v_min) / 2.0f - 90.0f;
  }
}

}  // namespace avoidance