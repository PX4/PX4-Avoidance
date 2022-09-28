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
#include <mutex>

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
  auto_loiter,
  none,
};

enum class MavCommand {
  MAV_CMD_NAV_LAND = 21,
  MAV_CMD_NAV_TAKEOFF,
  MAV_CMD_DO_CHANGE_SPEED = 178,
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

/**
* @brief struct to contain the parameters needed for the model based trajectory
*planning
* when MPC_AUTO_MODE is set to 1 (default) then all members are used for the
*jerk limited
* trajectory on the flight controller side
* when MPC_AUTO_MODE is set to 0, only up_accl, down_accl, xy_acc are used on
*the
* flight controller side
**/
struct ModelParameters {
  int param_mpc_auto_mode = -1;        // Auto sub-mode - 0: default line tracking, 1 jerk-limited trajectory
  float param_mpc_jerk_min = NAN;      // Velocity-based minimum jerk limit
  float param_mpc_jerk_max = NAN;      // Velocity-based maximum jerk limit
  float param_mpc_acc_up_max = NAN;    // Maximum vertical acceleration in velocity controlled modes upward
  float param_mpc_z_vel_max_up = NAN;  // Maximum vertical ascent velocity
  float param_mpc_acc_down_max = NAN;  // Maximum vertical acceleration in velocity controlled modes down
  float param_mpc_z_vel_max_dn = NAN;  // Maximum vertical descent velocity
  float param_mpc_acc_hor = NAN;       // Maximum horizontal acceleration for auto mode and
                                       // maximum deceleration for manual mode
  float param_mpc_xy_cruise = NAN;     // Desired horizontal velocity in mission
  float param_mpc_tko_speed = NAN;     // Takeoff climb rate
  float param_mpc_land_speed = NAN;    // Landing descend rate
  float param_mpc_yawrauto_max = NAN;

  float param_nav_acc_rad = NAN;

  // TODO: add estimator limitations for max speed and height

  float param_cp_dist = NAN;  // Collision Prevention distance to keep from obstacle. -1 for disabled
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
* @brief      determines whether a histogram cell lies inside the horizontal FOV
*             cell is considered inside if at least one edge lies inside
* @param[in]  vector of FOV structs defining current field of view
* @param[in]  idx, histogram cell column index
* @param[in]  position, current position
* @param[in]  yaw_fcu_frame, yaw orientation of the vehicle in global fcu frame
* @return     whether point is inside the FOV
**/
bool histogramIndexYawInsideFOV(const std::vector<FOV>& fov_vec, const int idx, Eigen::Vector3f position,
                                float yaw_fcu_frame);
bool histogramIndexYawInsideFOV(const FOV& fov, const int idx, Eigen::Vector3f position, float yaw_fcu_frame);

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

// todo: is this used?
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
* @brief     returns the correctly wrapped angle difference in degrees
* @param[in] the first angle in degrees
* @param[in] the second angle in degrees
* @returns   the angle between the two given angles [0, 180]
**/
float angleDifference(float a, float b);

/**
* @brief     computes an angular velocity to reach the desired_yaw
* @param[in] adesired_yaw  [rad]
* @param[in] curr_yaw  [rad]
* @returns   a scaled angular velocity to reach the desired yaw[rad/s]
**/
double getAngularVelocity(float desired_yaw, float curr_yaw);

/**
* @brief     transforms setpoints from ROS message to MavROS message
* @params[out] obst_avoid, setpoint in MavROS message form
* @params[in] pose, position and attitude setpoint computed by the planner
* @params[in] vel, velocity setpoint computed by the planner
**/
void transformToTrajectory(mavros_msgs::Trajectory& obst_avoid, geometry_msgs::PoseStamped pose,
                           geometry_msgs::Twist vel);

/**
* @brief      fills MavROS trajectory messages with NAN
* @param      point, setpoint to be filled with NAN
**/
void fillUnusedTrajectoryPoint(mavros_msgs::PositionTarget& point);

/**
* @brief     transforms bezier control points from ROS message to MavROS message
* @params[out] obst_avoid, control points in MavROS message form
* @params[in] control_points, control points in Eigen type
* @params[in] duration to execute the bezier curve
**/
void transformToBezier(mavros_msgs::Trajectory& obst_avoid, const std::array<Eigen::Vector4d, 5>& control_points,
                       double duration);

/**
* @brief     transforms bezier control point from Eigen to MavROS type
* @params[out] point_out, control point in MavROS message form
* @params[in]  point_in, control point in Eigen type
**/
void fillControlPoint(mavros_msgs::PositionTarget& point_out, const Eigen::Vector4d& point_in);
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

inline Eigen::Vector3f toEigen(const geometry_msgs::Point& p) {
  Eigen::Vector3f ev3(p.x, p.y, p.z);
  return ev3;
}

inline Eigen::Vector3f toEigen(const geometry_msgs::Vector3& v3) {
  Eigen::Vector3f ev3(v3.x, v3.y, v3.z);
  return ev3;
}

inline Eigen::Vector3f toEigen(const pcl::PointXYZ& p) {
  Eigen::Vector3f ev3(p.x, p.y, p.z);
  return ev3;
}

inline Eigen::Vector3f toEigen(const pcl::PointXYZI& p) {
  Eigen::Vector3f ev3(p.x, p.y, p.z);
  return ev3;
}

inline Eigen::Quaternionf toEigen(const geometry_msgs::Quaternion& gmq) {
  Eigen::Quaternionf eqf;
  eqf.x() = gmq.x;
  eqf.y() = gmq.y;
  eqf.z() = gmq.z;
  eqf.w() = gmq.w;
  return eqf;
}

inline geometry_msgs::Point toPoint(const Eigen::Vector3f& ev3) {
  geometry_msgs::Point gmp;
  gmp.x = ev3.x();
  gmp.y = ev3.y();
  gmp.z = ev3.z();
  return gmp;
}

inline geometry_msgs::Vector3 toVector3(const Eigen::Vector3f& ev3) {
  geometry_msgs::Vector3 gmv3;
  gmv3.x = ev3.x();
  gmv3.y = ev3.y();
  gmv3.z = ev3.z();
  return gmv3;
}

inline geometry_msgs::Quaternion toQuaternion(const Eigen::Quaternionf& eqf) {
  geometry_msgs::Quaternion q;
  q.x = eqf.x();
  q.y = eqf.y();
  q.z = eqf.z();
  q.w = eqf.w();
  return q;
}

inline pcl::PointXYZ toXYZ(const Eigen::Vector3f& ev3) {
  pcl::PointXYZ xyz;
  xyz.x = ev3.x();
  xyz.y = ev3.y();
  xyz.z = ev3.z();
  return xyz;
}

inline pcl::PointXYZI toXYZI(const Eigen::Vector3f& ev3, float intensity) {
  pcl::PointXYZI p;
  p.x = ev3.x();
  p.y = ev3.y();
  p.z = ev3.z();
  p.intensity = intensity;
  return p;
}

inline pcl::PointXYZI toXYZI(float x, float y, float z, float intensity) {
  pcl::PointXYZI p;
  p.x = x;
  p.y = y;
  p.z = z;
  p.intensity = intensity;
  return p;
}

inline pcl::PointXYZI toXYZI(const pcl::PointXYZ& xyz, float intensity) {
  pcl::PointXYZI p;
  p.x = xyz.x;
  p.y = xyz.y;
  p.z = xyz.z;
  p.intensity = intensity;
  return p;
}

inline geometry_msgs::Twist toTwist(const Eigen::Vector3f& l, const Eigen::Vector3f& a) {
  geometry_msgs::Twist gmt;
  gmt.linear = toVector3(l);
  gmt.angular = toVector3(a);
  return gmt;
}

inline geometry_msgs::PoseStamped toPoseStamped(const Eigen::Vector3f& ev3, const Eigen::Quaternionf& eq) {
  geometry_msgs::PoseStamped gmps;
  gmps.header.stamp = ros::Time::now();
  gmps.header.frame_id = "local_origin";
  gmps.pose.position = toPoint(ev3);
  gmps.pose.orientation = toQuaternion(eq);
  return gmps;
}

inline Eigen::Vector3f toNED(const Eigen::Vector3f& xyz_enu) {
  Eigen::Vector3f xyz_ned;
  xyz_ned.x() = xyz_enu.y();
  xyz_ned.y() = xyz_enu.x();
  xyz_ned.z() = -xyz_enu.z();
  return xyz_ned;
}

inline Eigen::Vector3f toENU(const Eigen::Vector3f& xyz_ned) {
  Eigen::Vector3f xyz_enu;
  xyz_enu.x() = xyz_ned.y();
  xyz_enu.y() = xyz_ned.x();
  xyz_enu.z() = -xyz_ned.z();
  return xyz_enu;
}

inline float yawToNEDdeg(const float yaw_enu) { return (90.f - yaw_enu); }

inline float yawToNEDrad(const float yaw_enu) { return (M_PI / 2.f - yaw_enu); }

inline float pitchtoNED(const float pitch_enu) { return (-pitch_enu); }

inline float yawToENUdeg(const float yaw_ned) { return (90.f - yaw_ned); }

inline float yawToENUrad(const float yaw_ned) { return (M_PI / 2.f - yaw_ned); }

inline float pitchToENU(const float pitch_ned) { return (-pitch_ned); }
Eigen::Quaterniond quaternionFromRPY(const Eigen::Vector3d& rpy);
Eigen::Quaterniond orientationToNED(const Eigen::Quaterniond& q);
Eigen::Quaterniond orientationToENU(const Eigen::Quaterniond& q);

static const Eigen::Vector3d NED_ENU_RPY(M_PI, 0, M_PI_2);
static const Eigen::Vector3d AIRCRAFT_BASELINK_RPY(M_PI, 0, 0);

}  // namespace avoidance

#endif  // COMMON_H
