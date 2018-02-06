#include "common.h"

float distance2DPolar(int e1, int z1, int e2, int z2){
  return sqrt(pow((e1-e2),2) + pow((z1-z2),2));
}

float computeL2Dist(geometry_msgs::PoseStamped pose, pcl::PointCloud<pcl::PointXYZ>::iterator pcl_it) {
  return sqrt(pow(pose.pose.position.x - pcl_it->x, 2) + pow(pose.pose.position.y - pcl_it->y, 2) + pow(pose.pose.position.z - pcl_it->z, 2));
}

float distance3DCartesian(geometry_msgs::Point a, geometry_msgs::Point b) {
  return sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) + (a.z-b.z)*(a.z-b.z));
}

// transform polar coordinates into Cartesian coordinates
geometry_msgs::Point fromPolarToCartesian(int e, int z, double radius, geometry_msgs::Point pos) {
  geometry_msgs::Point p;
  p.x = pos.x + radius * cos(e * (PI / 180)) * sin(z * (PI / 180));  //round
  p.y = pos.y + radius * cos(e * (PI / 180)) * cos(z * (PI / 180));
  p.z = pos.z + radius * sin(e * (PI / 180));

  return p;
}

double indexAngleDifference(int a, int b) {
return std::min(std::min(std::abs(a - b), std::abs(a - b - 360)), std::abs(a - b + 360));
}


// transform a 2D polar histogram direction in a 3D Catesian coordinate point
geometry_msgs::Vector3Stamped getWaypointFromAngle(int e, int z, geometry_msgs::Point pos) {
  geometry_msgs::Point p = fromPolarToCartesian(e, z, 1.0, pos);

  geometry_msgs::Vector3Stamped waypoint;
  waypoint.header.stamp = ros::Time::now();
  waypoint.header.frame_id = "/world";
  waypoint.vector.x = p.x;
  waypoint.vector.y = p.y;
  waypoint.vector.z = p.z;

  return waypoint;
}

// check if two points have the same altitude and yaw
bool hasSameYawAndAltitude(geometry_msgs::PoseStamped msg1, geometry_msgs::PoseStamped msg2){
  return abs(msg1.pose.orientation.z) >= abs(0.9*msg2.pose.orientation.z) && abs(msg1.pose.orientation.z) <= abs(1.1*msg2.pose.orientation.z)
         && abs(msg1.pose.orientation.w) >= abs(0.9*msg2.pose.orientation.w) && abs(msg1.pose.orientation.w) <= abs(1.1*msg2.pose.orientation.w)
         && abs(msg1.pose.position.z) >= abs(0.9*msg2.pose.position.z) && abs(msg1.pose.position.z) <= abs(1.1*msg2.pose.position.z);

}

double elevationIndexToAngle(int e, double res) {
  return e * res + res/2 - 90;
}

double azimuthIndexToAngle(int z, double res) {
  return z * res + res/2 - 180;
}

int azimuthAnglefromCartesian(double x, double y, double z, geometry_msgs::Point pos) {
  return floor(atan2(x - pos.x, y - pos.y) * 180.0 / PI);  //(-180. +180]
}

int elevationAnglefromCartesian(double x, double y, double z, geometry_msgs::Point pos) {
  return floor(atan((z - pos.z) / sqrt((x - pos.x) * (x - pos.x) + (y - pos.y) * (y - pos.y))) * 180.0 / PI);  //(-90.+90)
}

int elevationAngletoIndex(int e, int res) { //[-90,90]
  e += 90;
  e = e + (res - (e % res));  //[-80,+90]
  return e / res - 1;  //[0,17]
}

int azimuthAngletoIndex(int z, int res) { //[-90,90]
  z += 180;
  z = z + (res - (z % res));  //[-80,+90]
  return z / res - 1;  //[0,17]
}
