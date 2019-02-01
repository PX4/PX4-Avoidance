#ifndef COMMON_H
#define COMMON_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <Eigen/Core>

#include <pcl/point_types.h>

namespace avoidance {

struct PolarPoint {
  PolarPoint(float e_, float z_, float r_) : e(e_), z(z_), r(r_){};
  PolarPoint() : e(0.0f), z(0.0f), r(0.0f){};
  float e;
  float z;
  float r;
};

#define M_PI_F 3.14159265358979323846f
const float DEG_TO_RAD = M_PI_F / 180.f;
const float RAD_TO_DEG = 180.0f / M_PI_F;

/**
* @brief     calculates the distance between two polar points
* @param[in] p1 polar point
* @param[in] p2 polar point
* @returns   distance between the two points
**/
float distance2DPolar(const PolarPoint& p1, const PolarPoint& p2);

/**
* @brief     Convertes a polar point to a cartesian point and add it to a
*cartesian position
* @param[in] p_pol polar point to be converted to cartesian point
* @param[in] pos given cartesian position, from which to convert the polar point
* @returns   point in cartesian CS
**/
Eigen::Vector3f polarToCartesian(const PolarPoint& p_pol,
                                 const geometry_msgs::Point& pos);
float indexAngleDifference(float a, float b);
/**
* @brief     compute point in the histogram to a polar point
* @param[in] e evelation index in the histogram
* @param[in] z azimuth index in the histogram
* @param[in] res resolution of the histogram
* @param[in] radius of the polar point
* @param[out]polar point with elevation angle, azimuth angle and radius
**/
PolarPoint histogramIndexToPolar(int e, int z, int res, float radius);

/**
* @brief     Compute a cartesian point to polar CS
* @param[in] position Position of the location to which to compute the bearing
*            angles to.
* @param[in] origin Origin from which to compute the bearing vectors.
* @details   For a point given in cartesian x/y coordinates this is the
*            angle in degrees from the positive y-axis in (-180, 180].
*
* @returns   azimuth Angle in float degrees from the positive y-axis (-180, 180]
*            and elevation angle degrees (-90, 90]
**/

PolarPoint cartesianToPolar(const Eigen::Vector3f& pos,
                            const Eigen::Vector3f& origin);
PolarPoint cartesianToPolar(float x, float y, float z,
                            const Eigen::Vector3f& pos);
/**
* @brief     compute polar point to histogram index
* @param[in] p_pol with elevation, azimuth angle and radius
* @param[in] res resolution of the histogram in degrees
* @param[out]vector with x()=azimuth and y()=elevation
* @note      If there is an invalid input, the output index will be 0
**/

Eigen::Vector2i polarToHistogramIndex(const PolarPoint& p_pol, int res);
/**
* @brief     support function for polarToHistogramIndex
*            when abs(elevation) > 90, wrap elevation angle into valid
*            region and azimuth angle changes for +/-180 deg each time
* @param[in/out]p_pol Polar point with elevation angle [-90,90) and
*            azimuth angle [-180,180)
**/
void wrapPolar(PolarPoint& p_pol);
/**
* @brief     Compute the yaw angle between current position and point
* @returns   angle between two points in rad
**/
float nextYaw(const geometry_msgs::PoseStamped& u,
               const geometry_msgs::Point& v);

geometry_msgs::PoseStamped createPoseMsg(const geometry_msgs::Point& waypt,
                                         float yaw);

/**
* @brief     computes a speed using the upper and lower speed limit, as well as
*            current acceleration and velocity
* @param[in] max_vel upper limit for speed
* @param[in] min_vel lower limit for speed, currently always set as 0.0 hence
*            not used
* @param[in] slope hard coded as 1.0
* @param[in] v_old
* @param[in] elapsed time [s]
* @returns   speed within the given limits
**/
float velocityLinear(float max_vel, float slope, float v_old,
                      float elapsed);
/**
* @brief     wrappes the input angle in to plus minus PI space
* @param[in, out] angle to be wrapped  [rad]
**/
void wrapAngleToPlusMinusPI(float& angle);
/**
* @brief     wrappes the input angle in to plus minus 180 deg space
* @param[in, out] angle to be wrapped  [deg]
**/
void wrapAngleToPlusMinus180(float& angle);
/**
* @brief     computes an angular velocity to reach the desired_yaw
* @param[in] adesired_yaw  [rad]
* @param[in] curr_yaw  [rad]
* @returns   a scaled angular velocity to reach the desired yaw[rad/s]
**/
double getAngularVelocity(float desired_yaw, float curr_yaw);

Eigen::Vector3f toEigen(const geometry_msgs::Point& p);
Eigen::Vector3f toEigen(const geometry_msgs::Vector3& v3);
Eigen::Vector3f toEigen(const pcl::PointXYZ& xyz);

geometry_msgs::Point toPoint(const Eigen::Vector3f& ev3);
pcl::PointXYZ toXYZ(const Eigen::Vector3f& ev3);
}

#endif  // COMMON_H
