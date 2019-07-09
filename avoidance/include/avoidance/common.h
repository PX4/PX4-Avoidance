#ifndef COMMON_H
#define COMMON_H

#include "avoidance/histogram.h"  // needed for ALPHA_RES

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/Trajectory.h>
#include <tf/transform_listener.h>

namespace avoidance {

enum class MAV_STATE {
  MAV_STATE_UNINIT,
  MAV_STATE_BOOT,
  MAV_STATE_CALIBRATIN,
  MAV_STATE_STANDBY,
  MAV_STATE_ACTIVE,
  MAV_STATE_CRITICAL,
  MAV_STATE_EMERGENCY,
  MAV_STATE_POWEROFF,
  MAV_STATE_FLIGHT_TERMINATION,
};

enum class NavigationState {
  mission,
  auto_takeoff,
  auto_land,
  auto_rtl,
  auto_rtgs,
  offboard,
  none,
};

struct PolarPoint {
  PolarPoint(float e_, float z_, float r_) : e(e_), z(z_), r(r_){};
  PolarPoint() : e(0.0f), z(0.0f), r(0.0f){};
  float e;
  float z;
  float r;
};

/**
* Struct defining the Field of View of the sensor. This is defined by
* the current azimuth and elevation angles and the horizontal and vertical
* field of view of the sensor
*/
struct FOV {
  FOV() : yaw_deg(0.f), pitch_deg(0.f), h_fov_deg(0.f), v_fov_deg(0.f){};
  FOV(float y, float p, float h, float v) : yaw_deg(y), pitch_deg(p), h_fov_deg(h), v_fov_deg(v){};
  float yaw_deg;
  float pitch_deg;
  float h_fov_deg;
  float v_fov_deg;
};

#define M_PI_F 3.14159265358979323846f
#define WARN_UNUSED __attribute__((warn_unused_result))

const float DEG_TO_RAD = M_PI_F / 180.f;
const float RAD_TO_DEG = 180.0f / M_PI_F;

/**
* @brief      determines whether point is inside FOV
* @param[in]  vector of FOV structs defining current field of view
* @param[in]  p_pol, polar representation of the point in question
* @return     whether point is inside the FOV
**/
bool pointInsideFOV(const std::vector<FOV>& fov_vec, const PolarPoint& p_pol);
bool pointInsideFOV(const FOV& fov, const PolarPoint& p_pol);

/**
* @brief      determines whether point is inside the Yaw of the FOV
* @param[in]  vector of FOV structs defining current field of view
* @param[in]  p_pol, polar representation of the point in question
* @return     whether point is inside the FOV
**/
bool pointInsideYawFOV(const std::vector<FOV>& fov_vec, const PolarPoint& p_pol);
bool pointInsideYawFOV(const FOV& fov, const PolarPoint& p_pol);

/**
* @brief      compute in which FOV the current point lies
* @param[in]  vector of FOV defining the field of view of the drone
* @param[in]  polar point of the current orientation in question
* @param[out] index pointing to the camera in the FOV struct which contains the
*             current point
* @returns    boolean value if the point in question is in exactly one FOV
* @warning    This function returns false and sets the index to -1 if there is
*             no or more than one camera which sees the current point
**/
bool isInWhichFOV(const std::vector<FOV>& fov_vec, const PolarPoint& p_pol, int& idx);

/**
* @brief      determine whether the given point lies on the edge of the field
*             of view or between two adjacent cameras
* @param[in]  vector of FOV defining the field of view of the drone
* @param[in]  polar point of the current orientation in question
* @param[out] index of the camera in the FOV vector, indicating which FOV edge
*             it is on, if any. -1 if none
* @returns    boolean indicating whether the current point is on the edge of the
*             field of view
* @warning    This function returns false and sets the index to -1 if the point
*             is not on the edge of the fov
**/
bool isOnEdgeOfFOV(const std::vector<FOV>& fov_vec, const PolarPoint& p_pol, int& idx);

/**
* @brief     function returning a scale value depending on where a polar point
*            is relative to the field of view
* @param[in] vector of FOV structs defining the field of view
* @param[in] polar point in the fcu frame pointing in the direction we wish to
*            go
* @returns   a scale [0, 1] depending on whether the point in question can be
*            seen from here
**/
float scaleToFOV(const std::vector<FOV>& fov, const PolarPoint& p_pol);

/**
* @brief     calculates the distance between two polar points
* @param[in] p1 polar point
* @param[in] p2 polar point
* @returns   distance between the two points
**/
float distance2DPolar(const PolarPoint& p1, const PolarPoint& p2);

/**
* @brief     Converts a polar point in histogram convention to a cartesian point
*            and add it to a cartesian position
* @param[in] polar point in histogram convention to be converted
* @param[in] cartesian position, from which to convert the polar point
* @returns   point in cartesian CS
* @warning   The histogram convention means zero-azimuth is the positive y axis
*            with increasing azimuth in CW direction, while the elevation angle
*            increases for "upward looking" (contrary to pitch in FCU!)
**/
Eigen::Vector3f polarHistogramToCartesian(const PolarPoint& p_pol, const Eigen::Vector3f& pos);

/**
* @brief     Converts a polar point in fcu convention to a cartesian point and
*            add it to a cartesian position
* @param[in] polar point to be converted to cartesian point
* @param[in] given cartesian position, from which to convert the polar point
* @returns   point in cartesian coordinates
* @warning   The returned point is computed assuming the FCU convention on the
*            incoming polar point. This means the pitch is positive for forward
*            pitching of a quadrotor and the yaw is positive for CCW yaw motion
**/
Eigen::Vector3f polarFCUToCartesian(const PolarPoint& p_pol, const Eigen::Vector3f& pos);

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
* @brief     Compute polar vector in histogram convention between two cartesian
*            points
* @param[in] position Position of the location to which to compute the bearing
*            angles to.
* @param[in] origin Origin from which to compute the bearing vectors.
* @details   For a point given in cartesian x/y coordinates this is the
*            angle in degrees from the positive y-axis in (-180, 180].
*
* @returns   azimuth Angle in float degrees from the positive y-axis (-180, 180]
*            and elevation angle degrees (-90, 90]
**/
PolarPoint cartesianToPolarHistogram(const Eigen::Vector3f& pos, const Eigen::Vector3f& origin);
PolarPoint cartesianToPolarHistogram(float x, float y, float z, const Eigen::Vector3f& pos);

/**
* @brief     Compute the polar vector in FCU convention between two cartesian
*            points
* @param[in] endpoint of the polar vector
* @param[in] origin of the polar vector
* @returns   polar point in FCU convention that points from the given origin to
*            the given point
* @warning   the output adheres to the FCU convention: positive yaw is measured
*            CCW from the positive x-axis, and positve pitch is measured CCW
*            from the positve x-axis. (Positive pitch is pitching forward)
* @note      An overloaded function taking a pcl::PointXYZ assumes the origin
*            (0, 0, 0)
**/
PolarPoint cartesianToPolarFCU(const Eigen::Vector3f& pos, const Eigen::Vector3f& origin);
PolarPoint cartesianToPolarFCU(const pcl::PointXYZ& p);

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
float nextYaw(const Eigen::Vector3f& u, const Eigen::Vector3f& v);

void createPoseMsg(Eigen::Vector3f& out_waypt, Eigen::Quaternionf& out_q, const Eigen::Vector3f& in_waypt, float yaw);

/**
* @brief     Compute the yaw angle from a quaternion
* @returns   yaw angle in degrees
**/
float getYawFromQuaternion(const Eigen::Quaternionf q);

/**
* @brief     Compute the pitch angle from a quaternion
* @returns   pitch angle in degrees
**/
float getPitchFromQuaternion(const Eigen::Quaternionf q);

/**
* @brief     wrappes the input angle in to plus minus PI space
* @param[in] angle to be wrapped  [rad]
* @returns   wrapped angle [rad]
**/
float WARN_UNUSED wrapAngleToPlusMinusPI(float angle);
/**
* @brief     wrappes the input angle in to plus minus 180 deg space
* @param[in] angle to be wrapped  [deg]
* @returns   wrapped angle [deg]
**/
float WARN_UNUSED wrapAngleToPlusMinus180(float angle);
/**
* @brief     computes an angular velocity to reach the desired_yaw
* @param[in] adesired_yaw  [rad]
* @param[in] curr_yaw  [rad]
* @returns   a scaled angular velocity to reach the desired yaw[rad/s]
**/
double getAngularVelocity(float desired_yaw, float curr_yaw);

Eigen::Vector3f toEigen(const geometry_msgs::Point& p);
Eigen::Vector3f toEigen(const geometry_msgs::Vector3& v3);
Eigen::Vector3f toEigen(const pcl::PointXYZ& p);
Eigen::Vector3f toEigen(const pcl::PointXYZI& p);
Eigen::Quaternionf toEigen(const geometry_msgs::Quaternion& gmq);

geometry_msgs::Point toPoint(const Eigen::Vector3f& ev3);
geometry_msgs::Vector3 toVector3(const Eigen::Vector3f& ev3);
geometry_msgs::Quaternion toQuaternion(const Eigen::Quaternionf& qf3);
pcl::PointXYZ toXYZ(const Eigen::Vector3f& ev3);
pcl::PointXYZI toXYZI(const Eigen::Vector3f& ev3, float intensity);
pcl::PointXYZI toXYZI(float x, float y, float z, float intensity);
pcl::PointXYZI toXYZI(const pcl::PointXYZ& xyz, float intensity);
geometry_msgs::Twist toTwist(const Eigen::Vector3f& l, const Eigen::Vector3f& a);
geometry_msgs::PoseStamped toPoseStamped(const Eigen::Vector3f& p, const Eigen::Quaternionf& q);
/**
* @brief     transforms position setpoints from ROS message to MavROS message
* @params[out] obst_avoid, position setpoint in MavROS message form
* @params[in] pose, position setpoint computed by the planner
**/
void transformPoseToTrajectory(mavros_msgs::Trajectory& obst_avoid, geometry_msgs::PoseStamped pose);
/**
* @brief      transforms velocity setpoints from ROS message to MavROS
*             message
* @param[out] obst_avoid, velocity setpoint in MavROS message form
* @param[in]  vel, velocity setpoint computd by the planner
**/
void transformVelocityToTrajectory(mavros_msgs::Trajectory& obst_avoid, geometry_msgs::Twist vel);

/**
* @brief      fills MavROS trajectory messages with NAN
* @param      point, setpoint to be filled with NAN
**/
void fillUnusedTrajectoryPoint(mavros_msgs::PositionTarget& point);

/**
* @brief           This is a refactored version of the PCL library function
*                  "removeNaNFromPointCloud" to remove NAN values from the
*                  point cloud and compute the FOV
* @note            It operates in-place and iterates through the cloud once
* @param[in, out]  cloud The point cloud to be filtered in the camera frame
* @returns         a cloud containing the eight corners of the box containing
*                  all the points, in the same frame as the given point cloud
**/
pcl::PointCloud<pcl::PointXYZ> removeNaNAndGetMaxima(pcl::PointCloud<pcl::PointXYZ>& cloud);

/**
* @brief           Compute the FOV given a box of 8 points defining a box
* @param[in]       FOV to be updated
* @param[in]       point cloud containing 8 points which define a cube that
*                  contains all the points in a point cloud in the FCU frame
* @note            the FOV is only adjusted if the current cloud indicates a
*                  bigger FOV than previously thought
**/
void updateFOVFromMaxima(FOV& fov, const pcl::PointCloud<pcl::PointXYZ>& maxima);

}  // namespace avoidance

#endif  // COMMON_H
