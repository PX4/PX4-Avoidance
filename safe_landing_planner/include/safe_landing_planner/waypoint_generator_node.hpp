#pragma once
#include <ros/ros.h>
#include <ros/time.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/Trajectory.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/State.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


#include <dynamic_reconfigure/server.h>
#include <safe_landing_planner/WaypointGeneratorNodeConfig.h>
#include <safe_landing_planner/LSDGridMsg.h>
#include "grid.hpp"


#include <Eigen/Dense>

namespace avoidance {

  const std::vector<Eigen::Vector2f> exploration_pattern = {Eigen::Vector2f(1.f, 0.f), Eigen::Vector2f(1.f, 1.f),
            Eigen::Vector2f(0.f, 1.f), Eigen::Vector2f(-1.f, 1.f), Eigen::Vector2f(-1.f, 0.f), Eigen::Vector2f(-1.f, -1.f),
          Eigen::Vector2f(0.f, -1.f), Eigen::Vector2f(1.f, -1.f) };

  enum class LSDState {
    goTo,
    loiter,
    land,
    altitudeChange,
  };

  class WaypointGeneratorNode {

  public:
    WaypointGeneratorNode(const ros::NodeHandle& nh);
    ~WaypointGeneratorNode() = default;

    /**
    * @brief spins node
    **/
    void startNode();

  private:
    ros::NodeHandle nh_;

    ros::Timer cmdloop_timer_;
    std::unique_ptr<ros::AsyncSpinner> cmdloop_spinner_;

    double spin_dt_ = 0.1;
    float yaw_setpoint_ = NAN;
    float yaw_speed_setpoint_ = NAN;
    float loiter_yaw_ = NAN;
    float yaw_ = NAN;
    float beta_ = 0.9f;
    float landing_radius_ = 2.f;
    float can_land_thr_= 0.4f;
    float loiter_height_ = 4.f;
    float factor_exploration_ = 1.f;
    float vertical_range_error_ = 1.f;
    bool grid_received_ = false;
    bool is_land_waypoint_ = false;
    bool decision_taken_ = false;
    bool can_land_ = true;
    bool in_land_vertical_range_ = false;
    bool is_within_landing_radius_ = false;
    bool update_smoothing_size_ = false;
    bool explorarion_is_active_ = false;
    int smoothing_land_cell_ = 2;
    int start_seq_landing_decision_ = 0;
    int grid_lsd_seq_ = 0;
    int n_explored_pattern_ = -1;

    ros::Subscriber pose_sub_;
    ros::Subscriber trajectory_sub_;
    ros::Subscriber mission_sub_;

    ros::Subscriber grid_sub_;
    ros::Subscriber pos_index_sub_;
    ros::Subscriber state_sub_;

    ros::Publisher trajectory_pub_;
    ros::Publisher land_hysteresis_pub_;
    ros::Publisher marker_goal_pub_;

    Eigen::Vector3f position_ = Eigen::Vector3f(NAN, NAN, NAN);
    Eigen::Vector3f goal_ = Eigen::Vector3f(NAN, NAN, NAN);
    Eigen::Vector3f goal_visualization_ = Eigen::Vector3f::Zero();
    Eigen::Vector3f velocity_setpoint_ = Eigen::Vector3f(NAN, NAN, NAN);
    Eigen::Vector3f loiter_position_ = Eigen::Vector3f(NAN, NAN, NAN);
    Eigen::Vector3f exploration_anchor_ = Eigen::Vector3f(NAN, NAN, NAN);
    Eigen::Vector3f vel_sp = Eigen::Vector3f(NAN, NAN, NAN);
    Eigen::Vector2i pos_index_ = Eigen::Vector2i(NAN, NAN);
    Eigen::MatrixXf mean_ = Eigen::MatrixXf(40, 40);
    Eigen::MatrixXi land_ = Eigen::MatrixXi(40, 40);

    std::vector<float> can_land_hysteresis_;
    Grid grid_lsd_ = Grid(10.f, 1.f);
    LSDState lsd_state_ = LSDState::goTo;
    LSDState prev_lsd_state_ = LSDState::goTo;

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
    void positionCallback(const geometry_msgs::PoseStamped &msg);

    /**
    * @brief     callaback for setting the goal from the FCU Mission Waypoints
    * @param[in] msg, current and next position goals
    **/
    void trajectoryCallback(const mavros_msgs::Trajectory& msg);

    /**
    * @brief     callaback with the list of FCU Mission Items
    * @param[in] msg, list of mission items
    **/
    void missionCallback(const mavros_msgs::WaypointList& msg);

    /**
    * @brief     callaback with the grid calculated by the safe_landing_planner
    * @param[in] msg, grid
    **/
    void gridCallback(const safe_landing_planner::LSDGridMsg &msg);

    /**
    * @brief     callaback with the vehicle state
    * @param[in] msg, FCU vehicle state
    **/
    void stateCallback(const mavros_msgs::State &msg);

    /**
    * @brief     computes the setpoints to be sent to the FCU
    **/
    void calculateWaypoint();

    /**
    * @brief     decides if the desired setpoints received from the FCU should be overwritten
    *            or sent back because no intervention is needed
    * @returns   true, overwtite setpoints
    **/
    void updateLSDState();

    /**
    * @brief     publishes the computed waypoints to the FCU
    * @param[in] pos_sp, position setpoint
    * @param[in] vel_sp, velocity setpoint
    * @param[in] yaw_sp, yaw setpoint
    * @param[in] yaw_speed_sp, yaw speed setpoint
    **/
    void publishTrajectorySetpoints(const Eigen::Vector3f &pos_sp, const Eigen::Vector3f &vel_sp, float yaw_sp, float yaw_speed_sp);

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
