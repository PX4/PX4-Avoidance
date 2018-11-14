#include "common.h"

float distance2DPolar(int e1, int z1, int e2, int z2) {
  return sqrt(pow((e1 - e2), 2) + pow((z1 - z2), 2));
}

float computeL2Dist(geometry_msgs::Point position,
                    pcl::PointCloud<pcl::PointXYZ>::iterator pcl_it) {
  return sqrt(pow(position.x - pcl_it->x, 2) + pow(position.y - pcl_it->y, 2) +
              pow(position.z - pcl_it->z, 2));
}

float distance3DCartesian(geometry_msgs::Point a, geometry_msgs::Point b) {
  return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) +
              (a.z - b.z) * (a.z - b.z));
}

// transform polar coordinates into Cartesian coordinates
geometry_msgs::Point fromPolarToCartesian(int e, int z, double radius,
                                          geometry_msgs::Point pos) {
  geometry_msgs::Point p;
  p.x =
      pos.x + radius * cos(e * (M_PI / 180)) * sin(z * (M_PI / 180));  // round
  p.y = pos.y + radius * cos(e * (M_PI / 180)) * cos(z * (M_PI / 180));
  p.z = pos.z + radius * sin(e * (M_PI / 180));

  return p;
}

double indexAngleDifference(int a, int b) {
  return std::min(std::min(std::abs(a - b), std::abs(a - b - 360)),
                  std::abs(a - b + 360));
}

// transform a 2D polar histogram direction in a 3D Catesian coordinate point
geometry_msgs::Vector3Stamped getWaypointFromAngle(int e, int z,
                                                   geometry_msgs::Point pos) {
  geometry_msgs::Point p = fromPolarToCartesian(e, z, 1.0, pos);

  geometry_msgs::Vector3Stamped waypoint;
  waypoint.header.stamp = ros::Time::now();
  waypoint.header.frame_id = "/local_origin";
  waypoint.vector.x = p.x;
  waypoint.vector.y = p.y;
  waypoint.vector.z = p.z;

  return waypoint;
}

// check if two points have the same altitude and yaw
bool hasSameYawAndAltitude(geometry_msgs::PoseStamped old_wp,
                           geometry_msgs::Vector3Stamped new_wp, double new_yaw,
                           double old_yaw) {
  return abs(new_yaw) >= abs(0.9 * old_yaw) &&
         abs(new_yaw) <= abs(1.1 * old_yaw) &&
         abs(new_wp.vector.z) >= abs(0.9 * old_wp.pose.position.z) &&
         abs(new_wp.vector.z) <= abs(1.1 * old_wp.pose.position.z);
}

double elevationIndexToAngle(int e, double res) {
  return e * res + res / 2 - 90;
}

double azimuthIndexToAngle(int z, double res) {
  return z * res + res / 2 - 180;
}

int azimuthAnglefromCartesian(double x, double y, double z,
                              geometry_msgs::Point pos) {
  return floor(atan2(x - pos.x, y - pos.y) * 180.0 / M_PI);  //(-180. +180]
}

int elevationAnglefromCartesian(double x, double y, double z,
                                geometry_msgs::Point pos) {
  double den = sqrt((x - pos.x) * (x - pos.x) + (y - pos.y) * (y - pos.y));
  if (den == 0) {
    return 0;
  } else {
    return floor(atan((z - pos.z) / den) * 180.0 / M_PI);  //(-90.+90)
  }
}

int elevationAngletoIndex(int e, int res) {  //[-90,90]
  if (e == 90) {
    e = 89;
  }
  e += 90;
  e = e + (res - (e % res));  //[-80,+90]
  return e / res - 1;         //[0,17]
}

int azimuthAngletoIndex(int z, int res) {  //[-180,180]
  if (z == 180) {
    z = -180;
  }
  z += 180;
  z = z + (res - (z % res));  //[-80,+90]
  return z / res - 1;         //[0,17]
}

// calculate the yaw for the next waypoint
double nextYaw(geometry_msgs::PoseStamped u, geometry_msgs::Point v) {
  double dx = v.x - u.pose.position.x;
  double dy = v.y - u.pose.position.y;

  return atan2(dy, dx);
}

geometry_msgs::PoseStamped createPoseMsg(geometry_msgs::Point waypt,
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

void normalize(geometry_msgs::Point &p) {
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

double velocitySigmoid(double max_vel, double min_vel, double slope,
                       double v_old, double elapsed) {
  max_vel += 0.05;
  min_vel -= 0.05;
  v_old -= min_vel;
  double t_old = -1.0 / slope * log((max_vel - min_vel) / v_old - 1.0);
  double t_new = t_old + elapsed;
  double speed = min_vel + (max_vel - min_vel) / (1.0 + exp(-slope * t_new));
  return speed;
}

double velocityLinear(double max_vel, double min_vel, double slope,
                      double v_old, double elapsed) {
  v_old -= min_vel;
  double t_old = v_old / slope;
  double t_new = t_old + elapsed;
  double speed = min_vel + t_new * slope;
  return speed;
}

void wrapAngleToPlusMinusPI(double &angle) {
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
