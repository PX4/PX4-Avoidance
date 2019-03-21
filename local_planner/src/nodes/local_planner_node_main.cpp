#include "local_planner/local_planner.h"
#include "local_planner/common.h"
#include "local_planner/local_planner_node.h"
#include "local_planner/waypoint_generator.h"

#include <boost/algorithm/string.hpp>

int main(int argc, char** argv) {
  using namespace avoidance;
  ros::init(argc, argv, "local_planner_node");
  LocalPlannerNode Node(true);
  ros::Duration(2).sleep();
  ros::Time start_time = ros::Time::now();
  bool hover = false;
  bool planner_is_healthy = true;
  avoidanceOutput planner_output;
  Node.local_planner_->disable_rise_to_goal_altitude_ =
      Node.disable_rise_to_goal_altitude_;
  bool startup = true;
  bool callPx4Params = true;
  Node.status_msg_.state = (int)MAV_STATE::MAV_STATE_BOOT;

  std::thread worker(&LocalPlannerNode::threadFunction, &Node);

  // spin node, execute callbacks
  while (ros::ok()) {
    hover = false;

#ifdef DISABLE_SIMULATION
    startup = false;
#else
    // visualize world in RVIZ
    if (!Node.world_path_.empty() && startup) {
      visualization_msgs::MarkerArray marker_array;
      if (!visualizeRVIZWorld(Node.world_path_, marker_array)) {
        Node.world_pub_.publish(marker_array);
      }
      startup = false;
    }

#endif

    // Process callbacks & wait for a position update
    while (!Node.position_received_ && ros::ok()) {
      ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
    }

    // Check if all information was received
    ros::Time now = ros::Time::now();
    ros::Duration since_last_cloud = now - Node.last_wp_time_;
    ros::Duration since_start = now - start_time;

    Node.checkFailsafe(since_last_cloud, since_start, planner_is_healthy,
                       hover);

    // If planner is not running, update planner info and get last results
    Node.updatePlanner();

    // send waypoint
    if (!Node.never_run_ && planner_is_healthy) {
      Node.publishWaypoints(hover);
      if (!hover) Node.status_msg_.state = (int)MAV_STATE::MAV_STATE_ACTIVE;
    }
    else {
      for (size_t i = 0; i < Node.cameras_.size(); ++i) {
        // once the camera info have been set once, unsubscribe from topic
        Node.cameras_[i].camera_info_sub_.shutdown();
      }
    }

    Node.position_received_ = false;

    // publish system status
    if (now - Node.t_status_sent_ > ros::Duration(0.2)) Node.publishSystemStatus();

  }

  Node.should_exit_ = true;
  Node.data_ready_cv_.notify_all();
  worker.join();
  return 0;
}
