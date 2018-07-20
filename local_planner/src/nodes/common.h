#ifndef COMMON_H
#define COMMON_H

#include <math.h>
#include <fstream>
#include <iostream>
#include <limits>
#include <vector>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <tf/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

float distance2DPolar(int e1, int z1, int e2, int z2);
float computeL2Dist(geometry_msgs::Point position,
                    pcl::PointCloud<pcl::PointXYZ>::iterator pcl_it);
float distance3DCartesian(geometry_msgs::Point a, geometry_msgs::Point b);
geometry_msgs::Point fromPolarToCartesian(int e, int z, double radius,
                                          geometry_msgs::Point pos);
double indexAngleDifference(int a, int b);
geometry_msgs::Vector3Stamped getWaypointFromAngle(int e, int z,
                                                   geometry_msgs::Point pos);
bool hasSameYawAndAltitude(geometry_msgs::PoseStamped old_wp,
                           geometry_msgs::Vector3Stamped new_wp, double new_yaw,
                           double old_yaw);
double elevationIndexToAngle(int e, double res);
double azimuthIndexToAngle(int z, double res);
int azimuthAnglefromCartesian(double x, double y, double z,
                              geometry_msgs::Point pos);
int elevationAnglefromCartesian(double x, double y, double z,
                                geometry_msgs::Point pos);
int elevationAngletoIndex(int e, int res);
int azimuthAngletoIndex(int z, int res);
double nextYaw(geometry_msgs::PoseStamped u, geometry_msgs::Point v);
geometry_msgs::PoseStamped createPoseMsg(geometry_msgs::Point waypt,
                                         double yaw);
void normalize(geometry_msgs::Point &p);

double velocitySigmoid(double max_vel, double min_vel, double slope,
                       double v_old, double elapsed);
double velocityLinear(double max_vel, double min_vel, double slope,
                      double v_old, double elapsed);
void wrapAngleToPlusMinusPI(double &angle);
double getAngularVelocity(double desired_yaw, double curr_yaw);

#endif  // COMMON_H
