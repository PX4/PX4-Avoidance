#include "local_planner/local_planner_node.h"

using namespace avoidance;

int main(int argc, char** argv) {
  ros::init(argc, argv, "local_planner_node");
  ros::NodeHandle nh("~");
  ros::NodeHandle nh_private("");

  LocalPlannerNode Node(nh, nh_private, true);
  ros::Duration(2).sleep();

  std::thread worker(&LocalPlannerNode::threadFunction, &Node);

  std::thread worker_params(&LocalPlannerNode::checkPx4Parameters, &Node);
    ros::Duration timeout_termination =
        ros::Duration(Node.local_planner_->timeout_termination_);
    ros::Time start_query_position = ros::Time::now();
    bool sent_error = false;
      ros::Duration since_query = ros::Time::now() - start_query_position;
      if (since_query > timeout_termination && !sent_error) {
        // clang-format off
    	  ROS_WARN("\033[1;33m Planner abort: missing required data \n \033[0m");
    	  ROS_WARN("----------------------------- Debugging Info -----------------------------");
    	  ROS_WARN("Local planner has not received a position from FCU, check the following: ");
    	  ROS_WARN("1. Check cables connecting PX4 autopilot with onboard computer");
    	  ROS_WARN("2. Set PX4 parameter MAV_1_MODE to onbard or external vision");
    	  ROS_WARN("3. Set correct fcu_url in local_planner launch file:");
    	  ROS_WARN("   Example direct connection to serial port: /dev/ttyUSB0:921600");
    	  ROS_WARN("   Example connection over mavlink router: udp://:14540@localhost:14557");
    	  ROS_WARN("--------------------------------------------------------------------------");
        // clang-format on
        Node.status_msg_.state = (int)MAV_STATE::MAV_STATE_FLIGHT_TERMINATION;
        Node.publishSystemStatus();
        sent_error = true;
      }
  worker.join();
  worker_params.join();

  for (size_t i = 0; i < Node.cameras_.size(); ++i) {
    Node.cameras_[i].cloud_ready_cv_->notify_all();
    Node.cameras_[i].transform_thread_.join();
  }

  return 0;
}
