#pragma once
#include <ros/ros.h>
#include <ros/time.h>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Trajectory.h>
#include <ros/callback_queue.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int64MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <dynamic_reconfigure/server.h>
#include <safe_landing_planner/SLPGridMsg.h>
#include <safe_landing_planner/WaypointGeneratorNodeConfig.h>

#include <safe_landing_planner/waypoint_generator.hpp>

namespace avoidance {

class WaypointGeneratorNode final {
 public:
  WaypointGeneratorNode(const ros::NodeHandle& nh);
  ~WaypointGeneratorNode() = default;

  /**
  * @brief spins node
  **/
  void startNode();

 protected:
  WaypointGenerator waypointGenerator_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Timer cmdloop_timer_;
  std::unique_ptr<ros::AsyncSpinner> cmdloop_spinner_;
  ros::CallbackQueue cmdloop_queue_;

  ros::Subscriber pose_sub_;
  ros::Subscriber trajectory_sub_;

  ros::Subscriber grid_sub_;
  ros::Subscriber pos_index_sub_;
  ros::Subscriber state_sub_;

  ros::Publisher trajectory_pub_;
  ros::Publisher land_hysteresis_pub_;
  ros::Publisher marker_goal_pub_;

  bool grid_received_ = false;
  double spin_dt_ = 0.1;
  Eigen::Vector3f goal_visualization_ = Eigen::Vector3f::Zero();

  dynamic_reconfigure::Server<safe_landing_planner::WaypointGeneratorNodeConfig> server_;

  /**
  * @brief main loop callback
  * @param[in] event, event timing information
  **/
  void cmdLoopCallback(const ros::TimerEvent& event);

  /**
  * @brief     sets parameters from ROS parameter server
  * @param     config, struct containing all the parameters
  * @param     level, bitmask to group together reconfigurable parameters
  **/
  void dynamicReconfigureCallback(safe_landing_planner::WaypointGeneratorNodeConfig& config, uint32_t level);

  /**
  * @brif callback for vehicle position and orientation
  * @param[in] msg, pose message coming fro the FCU
  **/
  void positionCallback(const geometry_msgs::PoseStamped& msg);

  /**
  * @brief     callaback for setting the goal from the FCU Mission Waypoints
  * @param[in] msg, current and next position goals
  **/
  void trajectoryCallback(const mavros_msgs::Trajectory& msg);

  /**
  * @brief     callaback with the grid calculated by the safe_landing_planner
  * @param[in] msg, grid
  **/
  void gridCallback(const safe_landing_planner::SLPGridMsg& msg);

  /**
  * @brief     callaback with the vehicle state
  * @param[in] msg, FCU vehicle state
  **/
  void stateCallback(const mavros_msgs::State& msg);

  /**
  * @brief     publishes the computed waypoints to the FCU
  * @param[in] pos_sp, position setpoint
  * @param[in] vel_sp, velocity setpoint
  * @param[in] yaw_sp, yaw setpoint
  * @param[in] yaw_speed_sp, yaw speed setpoint
  **/
  void publishTrajectorySetpoints(const Eigen::Vector3f& pos_sp, const Eigen::Vector3f& vel_sp, float yaw_sp,
                                  float yaw_speed_sp);

  /**
  * @brief     fills unused waypoints with NANs
  * @param[in] point, unused waypoints
  **/
  void fillUnusedTrajectorySetpoints(mavros_msgs::PositionTarget& point);

  /**
  * @brief     visualize landing area decision grid in Rviz
  **/
  void landingAreaVisualization();

  /**
  * @brief     visualize goal in Rviz
  **/
  void goalVisualization();
};
}
