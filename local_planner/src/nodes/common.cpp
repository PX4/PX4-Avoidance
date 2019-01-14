#include "common.h"

#include <math.h>
#include <fstream>
#include <iostream>
#include <limits>
#include <vector>

#include <tf/transform_listener.h>
namespace avoidance {

float distance2DPolar(int e1, int z1, int e2, int z2) {
  return sqrt(pow((e1 - e2), 2) + pow((z1 - z2), 2));
}

float computeL2Dist(const geometry_msgs::Point& position,
                    const pcl::PointXYZ& xyz) {
  return sqrt(pow(position.x - xyz.x, 2) + pow(position.y - xyz.y, 2) +
              pow(position.z - xyz.z, 2));
}

float distance3DCartesian(const geometry_msgs::Point& a,
                          const geometry_msgs::Point& b) {
  return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) +
              (a.z - b.z) * (a.z - b.z));
}

// transform polar coordinates into Cartesian coordinates
geometry_msgs::Point fromPolarToCartesian(float e, float z, double radius,
                                          const geometry_msgs::Point& pos) {
  geometry_msgs::Point p;
  p.x =
      pos.x + radius * cos(e * (M_PI / 180.f)) * sin(z * (M_PI / 180.f));  // round
  p.y = pos.y + radius * cos(e * (M_PI / 180.f)) * cos(z * (M_PI / 180.f));
  p.z = pos.z + radius * sin(e * (M_PI / 180.f));

  return p;
}

double indexAngleDifference(float a, float b) {
  return std::min(std::min(std::abs(a - b), std::abs(a - b - 360.f)),
                  std::abs(a - b + 360.f));
}



double elevationIndexToAngle(int e, double res) {
  return e * res + res / 2 - 90;
}

double azimuthIndexToAngle(int z, double res) {
  return z * res + res / 2 - 180;
}

float azimuthAnglefromCartesian(const geometry_msgs::Point& position,
                              const geometry_msgs::Point& origin) {
  return azimuthAnglefromCartesian(position.x, position.y, origin);
}

float azimuthAnglefromCartesian(double x, double y,
                              const geometry_msgs::Point& pos) {
  return atan2(x - pos.x, y - pos.y) * 180.0 / M_PI;  //(-180. +180]
}

float elevationAnglefromCartesian(double x, double y, double z,
                                const geometry_msgs::Point& pos) {
  double den = sqrt((x - pos.x) * (x - pos.x) + (y - pos.y) * (y - pos.y));
  if (den == 0) {
    return 0;
  } else {
    return atan((z - pos.z) / den) * 180.0 / M_PI;  //(-90.+90)
  }
}

float elevationAnglefromCartesian(const geometry_msgs::Point& pos,
                                const geometry_msgs::Point& origin) {
  return elevationAnglefromCartesian(pos.x, pos.y, pos.z, origin);
}

int elevationAngletoIndex(float e, int res) {  //[-90,90]
  // Do some input-checks
  if(res <= 0.f || e < -90.f || e > 90.f){
    return 0.f;
  }
  
  if (e == 90.f) {
    e = 89;
  }
  e += 90;
  e = e + (res - ((int)e % res));  //[-80,+90]
  return floor(e / res )- 1;         //[0,17]
}

int azimuthAngletoIndex(float z, int res) {  //[-180,180]
  if(res <= 0.f || z < -180.f || z > 180.f){
    return 0.f;
  }
  if (z == 180.f) {
    z = -180;
  }
  z += 180;
  z = z + (res - ((int)z % res));  //[-80,+90]
  return z / res - 1;         //[0,17]
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

void normalize(geometry_msgs::Point& p) {
  double length = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
  if (length != 0) {
    p.x = p.x / length;
    p.y = p.y / length;
    p.z = p.z / length;
  } else {
    p.x = 0;
    p.y = 0;
    p.z = 0;
  }
}


double velocityLinear(double max_vel, double min_vel, double slope,
                      double v_old, double elapsed) {
  v_old -= min_vel;
  double t_old = v_old / slope;
  double t_new = t_old + elapsed;
  double speed = min_vel + t_new * slope;
  return speed;
}

void wrapAngleToPlusMinusPI(double& angle) {
  while (angle > M_PI) {
    angle -= 2 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2 * M_PI;
  }
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
  if (yaw_vel1 > 0) {
    yaw_vel2 = -(2 * M_PI - yaw_vel1);
  } else {
    yaw_vel2 = 2 * M_PI + yaw_vel1;
  }

  double vel;
  if (std::abs(yaw_vel1) < std::abs(yaw_vel2)) {
    vel = yaw_vel1;
  } else {
    vel = yaw_vel2;
  }

  return 0.5 * vel;
}

Eigen::Vector3f convert(const geometry_msgs::Point& p) {
  Eigen::Vector3f ev3(p.x, p.y, p.z);
  return ev3;
}

geometry_msgs::Point convert(const Eigen::Vector3f& p) {
  geometry_msgs::Point gmp;
  gmp.x = p.x();
  gmp.y = p.y();
  gmp.z = p.z();
  return gmp;
}
}
