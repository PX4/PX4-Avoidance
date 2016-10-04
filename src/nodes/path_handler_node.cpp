#include "path_handler_node.h"

namespace avoidance {

PathHandlerNode::PathHandlerNode() {

  ros::NodeHandle nh;

  trajectory_sub_ = nh.subscribe("/global_temp_path", 1, &PathHandlerNode::receivePath, this);
  ground_truth_sub_ = nh.subscribe("/mavros/local_position/pose", 1, &PathHandlerNode::positionCallback, this);

  mavros_waypoint_publisher_ = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
  current_waypoint_publisher_ = nh.advertise<geometry_msgs::PoseStamped>("/current_setpoint", 10);
  three_point_path_publisher_ = nh.advertise<nav_msgs::Path>("/three_point_path", 10);

  listener_.waitForTransform("/local_origin","/world", ros::Time(0), ros::Duration(3.0));

  current_goal_.header.frame_id="/world";
  current_goal_.pose.position.x = 0.5;
  current_goal_.pose.position.y = 0.5;
  current_goal_.pose.position.z = 3.5;

  current_goal_.pose.orientation.x = 0.0;
  current_goal_.pose.orientation.y = 0.0;
  current_goal_.pose.orientation.z = 0.0;
  current_goal_.pose.orientation.w = 1.0;
  last_pos_ = current_goal_;
  last_goal_ = current_goal_;

  ros::Rate rate(10);
  while(ros::ok())
  {
    rate.sleep();
    ros::spinOnce();

    auto x = current_goal_.pose.position.x - last_pos_.pose.position.x;
    auto y = current_goal_.pose.position.y - last_pos_.pose.position.y;
    auto z = current_goal_.pose.position.z - last_pos_.pose.position.z;
    tf::Vector3 vec(x,y,z);

    // If we are less than 1.0 away, then we should stop at the goal
    double new_len = vec.length() < 1.0 ? vec.length() : speed_;
    vec.normalize();
    vec *= new_len;

    auto setpoint = current_goal_;  // The intermediate position sent to Mavros
    setpoint.pose.position.x = last_pos_.pose.position.x + vec.getX();
    setpoint.pose.position.y = last_pos_.pose.position.y + vec.getY();
    setpoint.pose.position.z = last_pos_.pose.position.z + vec.getZ();

    // Publish setpoint for vizualization
    current_waypoint_publisher_.publish(setpoint);

    // Publish three point message
    if (path_.size() > 0) {
      nav_msgs::Path three_point_msg;
      three_point_msg.header.frame_id="/world";
      auto path_with_curr_pos_ = path_;
      path_with_curr_pos_.insert(path_with_curr_pos_.begin(), current_goal_);
      path_with_curr_pos_.insert(path_with_curr_pos_.begin(), last_goal_);
      three_point_msg.poses = path_with_curr_pos_;
      // three_point_msg.poses = filterPathCorners(path_with_curr_pos_);
      if (three_point_msg.poses.size() >= 3) {
        three_point_msg.poses.resize(3);
      }
      three_point_msg = smoothPath(three_point_msg);
      three_point_path_publisher_.publish(three_point_msg);
    }

    // setpoint = rotatePoseMsgToMavros(setpoint); // 90 deg fix
    listener_.transformPose("local_origin", ros::Time(0), setpoint, "world", setpoint);

    // Publish setpoint to Mavros
    mavros_waypoint_publisher_.publish(setpoint);
  }
}

PathHandlerNode::~PathHandlerNode() { }

void PathHandlerNode::receiveMessage(const geometry_msgs::PoseStamped & pose_msg) {

  // Not in use
  current_goal_ = pose_msg;
}

void PathHandlerNode::receivePath(const nav_msgs::Path & msg) {
  speed_ = min_speed_;
  path_.clear();
  for (int i=2; i < msg.poses.size(); ++i) {
    path_.push_back(msg.poses[i]);
  }
  if (path_.size() > 1) {
    current_goal_ = path_[0];
    last_goal_ = current_goal_;
  }
  else {
    ROS_INFO("  Received empty path\n");
  }
}

void PathHandlerNode::positionCallback(const geometry_msgs::PoseStamped & pose_msg) {

  // last_pos_ = rotatePoseMsgToWorld(last_pos_); // 90 deg fix
  listener_.transformPose("world", ros::Time(0), pose_msg, "local_origin", last_pos_);

  // Check if we are close enough to current goal to get the next part of the path
  if (path_.size() > 0 && std::abs(current_goal_.pose.position.x - last_pos_.pose.position.x) < 1 
                      && std::abs(current_goal_.pose.position.y - last_pos_.pose.position.y) < 1
                      && std::abs(current_goal_.pose.position.z - last_pos_.pose.position.z) < 1) {

    // TODO: get yawdiff(yaw1, yaw2)
    double yaw1 = tf::getYaw(current_goal_.pose.orientation);
    double yaw2 = tf::getYaw(last_pos_.pose.orientation);
    double yaw_diff = std::abs(yaw2 - yaw1);
    yaw_diff -= std::floor(yaw_diff / (2*M_PI)) * (2*M_PI);
    // double max_yaw_diff = M_PI / 8.0;
    double max_yaw_diff = M_PI / 4.0;
    if (yaw_diff < max_yaw_diff || yaw_diff  > 2*M_PI - max_yaw_diff){
      // If we are facing the right direction, then pop the first point of the path
      last_goal_ = current_goal_;
      current_goal_ = path_[0];
      path_.erase(path_.begin());

      // If we are keeping the same direction and height increase speed
      if (path_.size() > std::floor(speed_) && hasSameYawAndAltitude(current_goal_.pose, path_[std::floor(speed_)].pose)) {
        speed_ = std::min(max_speed_, speed_+0.1);
      }
      else {
        speed_ = min_speed_;
      }
    }

  }
}

} // namespace avoidance 

int main(int argc, char** argv) {
  ros::init(argc, argv, "path_handler_node");
  avoidance::PathHandlerNode path_handler_node;
  ros::spin();
  return 0;
}
