#ifndef COMMON_H
#define COMMON_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

namespace avoidance {

float distance2DPolar(int e1, int z1, int e2, int z2);
float computeL2Dist(const geometry_msgs::Point& position,
                    const pcl::PointXYZ& xyz);
float distance3DCartesian(const geometry_msgs::Point& a,
                          const geometry_msgs::Point& b);
geometry_msgs::Point fromPolarToCartesian(int e, int z, double radius,
                                          const geometry_msgs::Point& pos);
double indexAngleDifference(int a, int b);
geometry_msgs::Vector3Stamped getWaypointFromAngle(
    int e, int z, const geometry_msgs::Point& pos);
bool hasSameYawAndAltitude(const geometry_msgs::PoseStamped& old_wp,
                           const geometry_msgs::Vector3Stamped& new_wp,
                           double new_yaw, double old_yaw);
double elevationIndexToAngle(int e, double res);
double azimuthIndexToAngle(int z, double res);
int azimuthAnglefromCartesian(const geometry_msgs::Point& position,
                              const geometry_msgs::Point& origin);
int azimuthAnglefromCartesian(double x, double y,
                              const geometry_msgs::Point& pos);
int elevationAnglefromCartesian(const geometry_msgs::Point& pos,
                                const geometry_msgs::Point& origin);
int elevationAnglefromCartesian(double x, double y, double z,
                                const geometry_msgs::Point& pos);
int elevationAngletoIndex(int e, int res);
int azimuthAngletoIndex(int z, int res);
double nextYaw(const geometry_msgs::PoseStamped& u,
               const geometry_msgs::Point& v);
geometry_msgs::PoseStamped createPoseMsg(const geometry_msgs::Point& waypt,
                                         double yaw);
void normalize(geometry_msgs::Point& p);

double velocitySigmoid(double max_vel, double min_vel, double slope,
                       double v_old, double elapsed);
double velocityLinear(double max_vel, double min_vel, double slope,
                      double v_old, double elapsed);
void wrapAngleToPlusMinusPI(double& angle);
double getAngularVelocity(double desired_yaw, double curr_yaw);
}

#endif  // COMMON_H
