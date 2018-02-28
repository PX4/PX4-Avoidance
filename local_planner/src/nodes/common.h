#ifndef COMMON_H
#define COMMON_H

#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>
#include <limits>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>

#include <tf/transform_listener.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

#define PI 3.14159265
#define alpha_res 6
#define grid_length_z 360/alpha_res
#define grid_length_e 180/alpha_res
#define age_lim 100
#define min_bin 1.5
#define h_fov 59.0
#define v_fov 46.0
#define inf  std::numeric_limits<double>::infinity()

float distance2DPolar(int e1, int z1, int e2, int z2);
float computeL2Dist(geometry_msgs::Point position, pcl::PointCloud<pcl::PointXYZ>::iterator pcl_it);
float distance3DCartesian(geometry_msgs::Point a, geometry_msgs::Point b);
geometry_msgs::Point fromPolarToCartesian(int e, int z, double radius, geometry_msgs::Point pos);
double indexAngleDifference(int a, int b);
geometry_msgs::Vector3Stamped getWaypointFromAngle(int e, int z, geometry_msgs::Point pos);
bool hasSameYawAndAltitude(geometry_msgs::PoseStamped old_wp, geometry_msgs::Vector3Stamped new_wp, double new_yaw, double old_yaw);
double elevationIndexToAngle(int e, double res);
double azimuthIndexToAngle(int z, double res);
int azimuthAnglefromCartesian(double x, double y, double z, geometry_msgs::Point pos);
int elevationAnglefromCartesian(double x, double y, double z, geometry_msgs::Point pos);
int elevationAngletoIndex(int e, int res);
int azimuthAngletoIndex(int z, int res);
double nextYaw(geometry_msgs::PoseStamped u, geometry_msgs::Vector3Stamped v, double last_yaw);
geometry_msgs::PoseStamped createPoseMsg(geometry_msgs::Vector3Stamped waypt, double yaw);


#endif // COMMON_H
