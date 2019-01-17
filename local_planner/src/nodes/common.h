#ifndef COMMON_H
#define COMMON_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

namespace avoidance {

float distance2DPolar(int e1, int z1, int e2, int z2);

/**
* @brief     Convertes the point from polar CS to cartesian CS
* @param[in] e elevation angle (-90, 90]
* @param[in] z azimuth angle (-180,180]
* @param[in] radius
* @param[in] pos Position from which to convert the point
* @returns   point in cartesian CS
**/
Eigen::Vector3f fromPolarToCartesian(float e, float z, double radius,
                                     const geometry_msgs::Point& pos);
double indexAngleDifference(float a, float b);

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
float azimuthAnglefromCartesian(const Eigen::Vector3f& position,
                                const Eigen::Vector3f& origin);
float azimuthAnglefromCartesian(double x, double y, const Eigen::Vector3f& pos);
/**
* @brief   Compute the elevation angle for a point given in cartesian
*coordinates
* @note    Output is in degrees (-90, 90)
**/
float elevationAnglefromCartesian(const Eigen::Vector3f& pos,
                                  const Eigen::Vector3f& origin);
float elevationAnglefromCartesian(double x, double y, double z,
                                  const Eigen::Vector3f& pos);
/**
* @brief     Checks if the computed histogram index given an elevation angle and
*resolution is valid
* @param[in] e Elevation angle in degrees
* @param[in] res resolution of the histogram in degrees
* @param[in] index Index of the histogram cell for the given elevation
* @returns   true or
* @note      If there is an invalid input, the output index will be 0
**/
int elevationAngletoIndex(float e, int res);
int azimuthAngletoIndex(float z, int res);

/**
* @brief     Compute the yaw angle between current position and point
* @returns   angle between two points in rad
**/
double nextYaw(const geometry_msgs::PoseStamped& u,
               const geometry_msgs::Point& v);

geometry_msgs::PoseStamped createPoseMsg(const geometry_msgs::Point& waypt,
                                         double yaw);

/**
* @brief     computes a speed using the upper and lower speed limit, as well as
*current acceleration and velocity
* @param[in] max_vel upper limit for speed
* @param[in] min_vel lower limit for speed, currently always set as 0.0 hence
*not used
* @param[in] slope hard coded as 1.0
* @param[in] v_old
* @param[in] elapsed time [s]
* @returns   speed within the given limits
**/
double velocityLinear(double max_vel, double slope, double v_old,
                      double elapsed);
/**
* @brief     wrappes the input angle in to plus minus PI space
* @param[in, out] angle to be wrapped  [rad]
**/
void wrapAngleToPlusMinusPI(double& angle);

/**
* @brief     computes an angular velocity to reach the desired_yaw
* @param[in] adesired_yaw  [rad]
* @param[in] curr_yaw  [rad]
* @returns   a scaled angular velocity to reach the desired yaw[rad/s]
**/
double getAngularVelocity(double desired_yaw, double curr_yaw);

Eigen::Vector3f toEigen(const geometry_msgs::Point& p);
Eigen::Vector3f toEigen(const pcl::PointXYZ& xyz);

geometry_msgs::Point toPoint(const Eigen::Vector3f& ev3);
pcl::PointXYZ toXYZ(const Eigen::Vector3f& ev3);
}

#endif  // COMMON_H
