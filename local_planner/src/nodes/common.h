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

/**
* @brief     Compute the bearing angle of a given vector relative to an
*            origin.
* @param[in] position Position of the location to which to compute the bearing
*            angle to.
* @param[in] origin Origin from which to compute the bearing vector.
* @details   For a point given in cartesian x/y coordinates this is the
*            angle in degrees from the positive y-axis in (-180, 180].
*
* @returns   Angle in integer degrees from the positive y-axis (-180, 180]
* @warning   If the origin and the position coincide, the output is 0 degrees
**/
int azimuthAnglefromCartesian(const geometry_msgs::Point& position,
                              const geometry_msgs::Point& origin);
int azimuthAnglefromCartesian(double x, double y,
                              const geometry_msgs::Point& pos);

/**
* @brief   Compute the elevation angle for a point given in cartesian coordinates
* @note    Output is in degrees (-90, 90)
* @warning For the poles on the sphere, the Output is 0 degrees.
*          (As opposed to the expected +/- 90)
**/
int elevationAnglefromCartesian(const geometry_msgs::Point& pos,
                                const geometry_msgs::Point& origin);
int elevationAnglefromCartesian(double x, double y, double z,
                                const geometry_msgs::Point& pos);
/**
* @brief     Compute the histogram index given an elevation angle and resolution
* @param[in] e Elevation angle in degrees
* @param[in] res resolution of the histogram in degrees
* @returns   Index of the histogram cell for the given elevation
* @note      If there is an invalid input, the output index will be 0
**/
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

Eigen::Vector3f convert(const geometry_msgs::Point& p);
geometry_msgs::Point convert(const Eigen::Vector3f& p);
}

#endif  // COMMON_H
