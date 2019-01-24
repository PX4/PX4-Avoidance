#ifndef COMMON_H
#define COMMON_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <Eigen/Core>

#include <pcl/point_types.h>

namespace avoidance {

struct PolarPoint {
  float e;
  float z;
  float r;
};

float distance2DPolar(const PolarPoint& p1, const PolarPoint& p2);

/**
* @brief     Convertes the point from polar CS to cartesian CS
* @param[in] p_pol polar point
* @param[in] pos Position from which to convert the point
* @returns   point in cartesian CS
**/
Eigen::Vector3f PolarToCartesian(const PolarPoint& p_pol,
                                 const geometry_msgs::Point& pos);
double indexAngleDifference(float a, float b);

PolarPoint HistogramIndexToPolar(int e, int z, int res,
                                 const float& radius);

/**
* @brief     Compute the polar point to an
*            origin.
* @param[in] position Position of the location to which to compute the bearing
*            angle to.
* @param[in] origin Origin from which to compute the bearing vector.
* @details   For a point given in cartesian x/y coordinates this is the
*            angle in degrees from the positive y-axis in (-180, 180].
*
* @returns   azimuth Angle in float degrees from the positive y-axis (-180, 180] and elevation angle degrees (-90, 90]
* @warning   If the origin and the position coincide, the output is 0 degrees
**/



PolarPoint CartesianToPolar(const Eigen::Vector3f& pos,
                            const Eigen::Vector3f& origin);
PolarPoint CartesianToPolar(double x, double y, double z,
                            const Eigen::Vector3f& pos);
/**
* @brief     compute polar point to histogram index
* @param[in] p_pol with elevation, azimuth angle and radius
* @param[in] res resolution of the histogram in degrees
* @param[out] eigenvector with x()=azimuth and y()=elevation
* @note      If there is an invalid input, the output index will be 0
**/

Eigen::Vector2i PolarToHistogramIndex(const PolarPoint& p_pol, int res);

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
