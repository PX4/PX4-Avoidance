#include "avoidance/avoidance_node.h"

namespace avoidance {

AvoidanceNode::AvoidanceNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), cmdloop_dt_(0.1), statusloop_dt_(0.2) {
  position_received_ = true;
  should_exit_ = false;

  timeout_termination_ = 15;
  timeout_critical_ = 0.5;
  timeout_startup_ = 5.0;

  mission_item_speed_ = NAN;
  param_cb_mutex_.reset(new std::mutex);
}

AvoidanceNode::~AvoidanceNode() {}

void AvoidanceNode::init() {
  mavros_system_status_pub_ = nh_.advertise<mavros_msgs::CompanionProcessStatus>("mavros/companion_process/status", 1);
  px4_param_sub_ = nh_.subscribe("mavros/param/param_value", 1, &AvoidanceNode::px4ParamsCallback, this);
  mission_sub_ = nh_.subscribe("mavros/mission/waypoints", 1, &AvoidanceNode::missionCallback, this);
  get_px4_param_client_ = nh_.serviceClient<mavros_msgs::ParamGet>("mavros/param/get");

  ros::TimerOptions cmdlooptimer_options(ros::Duration(cmdloop_dt_),
                                         boost::bind(&AvoidanceNode::cmdLoopCallback, this, _1), &cmdloop_queue_);
  cmdloop_timer_ = nh_.createTimer(cmdlooptimer_options);

  ros::TimerOptions statuslooptimer_options(
      ros::Duration(statusloop_dt_), boost::bind(&AvoidanceNode::statusLoopCallback, this, _1), &statusloop_queue_);
  statusloop_timer_ = nh_.createTimer(statuslooptimer_options);

  setSystemStatus(MAV_STATE::MAV_STATE_BOOT);

  cmdloop_spinner_.reset(new ros::AsyncSpinner(1, &cmdloop_queue_));
  cmdloop_spinner_->start();
  statusloop_spinner_.reset(new ros::AsyncSpinner(1, &statusloop_queue_));
  statusloop_spinner_->start();

  worker_ = std::thread(&AvoidanceNode::checkPx4Parameters, this);
}

void AvoidanceNode::cmdLoopCallback(const ros::TimerEvent& event) {}

void AvoidanceNode::statusLoopCallback(const ros::TimerEvent& event) { publishSystemStatus(); }

void AvoidanceNode::setSystemStatus(MAV_STATE state) { companion_state_ = state; }

MAV_STATE AvoidanceNode::getSystemStatus() { return companion_state_; }

// Publish companion process status
void AvoidanceNode::publishSystemStatus() {
  mavros_msgs::CompanionProcessStatus status_msg;
  status_msg.header.stamp = ros::Time::now();
  status_msg.component = 196;  // MAV_COMPONENT_ID_AVOIDANCE
  status_msg.state = (int)companion_state_;

  mavros_system_status_pub_.publish(status_msg);
}

void AvoidanceNode::checkFailsafe(ros::Duration since_last_cloud, ros::Duration since_start, bool& hover) {
  ros::Duration timeout_termination = ros::Duration(timeout_termination_);
  ros::Duration timeout_critical = ros::Duration(timeout_critical_);
  ros::Duration timeout_startup = ros::Duration(timeout_startup_);

  if (since_last_cloud > timeout_termination && since_start > timeout_termination) {
    setSystemStatus(MAV_STATE::MAV_STATE_FLIGHT_TERMINATION);
    ROS_WARN("\033[1;33m Planner abort: missing required data \n \033[0m");
  } else {
    if (since_last_cloud > timeout_critical && since_start > timeout_startup) {
      if (position_received_) {
        hover = true;
        setSystemStatus(MAV_STATE::MAV_STATE_CRITICAL);
        std::string not_received = "";
      } else {
        ROS_WARN("\033[1;33m Pointcloud timeout: No position received, no WP to output.... \n \033[0m");
      }
    } else {
      if (!hover) setSystemStatus(MAV_STATE::MAV_STATE_ACTIVE);
    }
  }
}

void AvoidanceNode::px4ParamsCallback(const mavros_msgs::Param& msg) {
  // collect all px4_ parameters needed for model based trajectory planning
  // when adding new parameter to the struct ModelParameters,
  // add new else if case with correct value type
  auto parse_param_f = [&msg](const std::string& name, float& val) -> bool {
    if (msg.param_id == name) {
      ROS_INFO("parameter %s is set from  %f to %f \n", name.c_str(), val, msg.value.real);
      val = msg.value.real;
      return true;
    }
    return false;
  };

  auto parse_param_i = [&msg](const std::string& name, int& val) -> bool {
    if (msg.param_id == name) {
      ROS_INFO("parameter %s is set from %i to %li \n", name.c_str(), val, msg.value.integer);
      val = msg.value.integer;
      return true;
    }
    return false;
  };

  // clang-format off
  std::lock_guard<std::mutex> lck(*(param_cb_mutex_));
  parse_param_f("MPC_ACC_DOWN_MAX", px4_.param_mpc_acc_down_max) ||
  parse_param_f("MPC_ACC_HOR", px4_.param_mpc_acc_hor) ||
  parse_param_f("MPC_ACC_UP_MAX", px4_.param_mpc_acc_up_max) ||
  parse_param_i("MPC_AUTO_MODE", px4_.param_mpc_auto_mode) ||
  parse_param_f("MPC_JERK_MIN", px4_.param_mpc_jerk_min) ||
  parse_param_f("MPC_JERK_MAX", px4_.param_mpc_jerk_max) ||
  parse_param_f("MPC_LAND_SPEED", px4_.param_mpc_land_speed) ||
  parse_param_f("MPC_TKO_SPEED", px4_.param_mpc_tko_speed) ||
  parse_param_f("MPC_XY_CRUISE", px4_.param_mpc_xy_cruise) ||
  parse_param_f("MPC_Z_VEL_MAX_DN", px4_.param_mpc_z_vel_max_dn) ||
  parse_param_f("MPC_Z_VEL_MAX_UP", px4_.param_mpc_z_vel_max_up) ||
  parse_param_f("CP_DIST", px4_.param_cp_dist) ||
  parse_param_f("NAV_ACC_RAD", px4_.param_nav_acc_rad) ||
  parse_param_f("MPC_YAWRAUTO_MAX", px4_.param_mpc_yawrauto_max);
  // clang-format on
}

void AvoidanceNode::checkPx4Parameters() {
  auto& client = get_px4_param_client_;
  auto request_param = [&client](const std::string& name, float& val) {
    mavros_msgs::ParamGet req;
    req.request.param_id = name;
    if (client.call(req) && req.response.success) {
      val = req.response.value.real;
    }
  };
  while (!should_exit_) {
    bool is_param_not_initialized = true;
    {
      std::lock_guard<std::mutex> lck(*(param_cb_mutex_));
      request_param("MPC_ACC_HOR", px4_.param_mpc_acc_hor);
      request_param("MPC_ACC_DOWN_MAX", px4_.param_mpc_acc_down_max);
      request_param("MPC_ACC_UP_MAX", px4_.param_mpc_acc_up_max);
      request_param("MPC_XY_CRUISE", px4_.param_mpc_xy_cruise);
      request_param("MPC_Z_VEL_MAX_DN", px4_.param_mpc_z_vel_max_dn);
      request_param("MPC_Z_VEL_MAX_UP", px4_.param_mpc_z_vel_max_up);
      request_param("CP_DIST", px4_.param_cp_dist);
      request_param("MPC_LAND_SPEED", px4_.param_mpc_land_speed);
      request_param("MPC_JERK_MAX", px4_.param_mpc_jerk_max);
      request_param("NAV_ACC_RAD", px4_.param_nav_acc_rad);
      request_param("MPC_YAWRAUTO_MAX", px4_.param_mpc_yawrauto_max);

      is_param_not_initialized =
          !std::isfinite(px4_.param_mpc_xy_cruise) || !std::isfinite(px4_.param_cp_dist) ||
          !std::isfinite(px4_.param_mpc_land_speed) || !std::isfinite(px4_.param_nav_acc_rad) ||
          !std::isfinite(px4_.param_mpc_acc_hor) || !std::isfinite(px4_.param_mpc_jerk_max) ||
          !std::isfinite(px4_.param_mpc_acc_down_max) || !std::isfinite(px4_.param_mpc_acc_up_max) ||
          !std::isfinite(px4_.param_mpc_z_vel_max_dn) || !std::isfinite(px4_.param_mpc_z_vel_max_up) ||
          !std::isfinite(px4_.param_mpc_yawrauto_max);
    }

    if (is_param_not_initialized) {
      std::this_thread::sleep_for(std::chrono::seconds(5));
    } else {
      std::this_thread::sleep_for(std::chrono::seconds(30));
    }
  }
}

void AvoidanceNode::missionCallback(const mavros_msgs::WaypointList& msg) {
  for (int index = 0; index < msg.waypoints.size(); index++) {
    if (msg.waypoints[index].is_current) {
      for (int i = index; i >= 0; i--) {
        if (msg.waypoints[i].command == static_cast<int>(MavCommand::MAV_CMD_DO_CHANGE_SPEED) &&
            (msg.waypoints[i].param1 - 1.0f) < FLT_MIN && msg.waypoints[i].param2 > 0.0f) {
          // 1MAV_CMD_DO_CHANGE_SPEED, speed type: ground speed, speed valid
          mission_item_speed_ = msg.waypoints[i].param2;
          break;
        }
      }
      break;
    }
  }
}

ModelParameters AvoidanceNode::getPX4Parameters() const {
  std::lock_guard<std::mutex> lck(*(param_cb_mutex_));
  ModelParameters px4 = px4_;
  return px4;
}
}
