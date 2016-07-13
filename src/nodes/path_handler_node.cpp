#include "path_handler_node.h"

namespace avoidance {

PathHandlerNode::PathHandlerNode() {

  ros::NodeHandle nh;

  trajectory_sub_ = nh.subscribe("/global_path", 1, &PathHandlerNode::ReceivePath, this);
  ground_truth_sub_ = nh.subscribe("/mavros/local_position/pose", 1, &PathHandlerNode::PositionCallback, this);

  mavros_waypoint_publisher_ = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
  current_waypoint_publisher_ = nh.advertise<geometry_msgs::PoseStamped>("/current_setpoint", 10);
  three_point_path_publisher_ = nh.advertise<nav_msgs::Path>("/three_point_path", 10);

  listener_.waitForTransform("/local_origin","/world", ros::Time(0), ros::Duration(3.0));

  currentGoal.header.frame_id="/world";
  currentGoal.pose.position.x = 0.5;
  currentGoal.pose.position.y = 0.5;
  currentGoal.pose.position.z = 3.5;

  currentGoal.pose.orientation.x = 0.0;
  currentGoal.pose.orientation.y = 0.0;
  currentGoal.pose.orientation.z = 0.0;
  currentGoal.pose.orientation.w = 1.0;
  lastPos = currentGoal;

  ros::Rate r(10);
  while(ros::ok())
  {
    r.sleep();
    ros::spinOnce();

    auto x = currentGoal.pose.position.x - lastPos.pose.position.x;
    auto y = currentGoal.pose.position.y - lastPos.pose.position.y;
    auto z = currentGoal.pose.position.z - lastPos.pose.position.z;
    tf::Vector3 vec(x,y,z);

    // If we are less than 1.0 away, then we should stop at the goal
    double newLen = vec.length() < 1.0 ? vec.length() : speed;
    vec.normalize();
    vec *= newLen;

    auto setpoint = currentGoal;  // The intermediate position sent to Mavros
    setpoint.pose.position.x = lastPos.pose.position.x + vec.getX();
    setpoint.pose.position.y = lastPos.pose.position.y + vec.getY();
    setpoint.pose.position.z = lastPos.pose.position.z + vec.getZ();

    // Publish setpoint for vizualization
    current_waypoint_publisher_.publish(setpoint);

    // Publish three point message
    if (path.size() > 2) {
      nav_msgs::Path threePointMsg;
      threePointMsg.header.frame_id="/world";
      auto pathWithCurrPos = path;
      pathWithCurrPos.insert(pathWithCurrPos.begin(), currentGoal);
      threePointMsg.poses = filterPathCorners(pathWithCurrPos);
      if (threePointMsg.poses.size() >= 3) {
        threePointMsg.poses.resize(3);
      }
      three_point_path_publisher_.publish(threePointMsg);
    }

    // setpoint = rotatePoseMsgToMavros(setpoint); // 90 deg fix
    listener_.transformPose("local_origin", ros::Time(0), setpoint, "world", setpoint);

    // Publish setpoint to Mavros
    mavros_waypoint_publisher_.publish(setpoint);
  }
}

PathHandlerNode::~PathHandlerNode() { }

void PathHandlerNode::ReceiveMessage(const geometry_msgs::PoseStamped & pose_msg) {

  // Not in use
  currentGoal = pose_msg;
}

void PathHandlerNode::ReceivePath(const nav_msgs::Path & msg) {
  speed = 1.0;
  path.clear();
  for (int i=2; i < msg.poses.size(); ++i) {
    path.push_back(msg.poses[i]);
  }
  if (path.size() > 1) {
    currentGoal = path[0];
  }
  else {
    ROS_INFO("  Received empty path\n");
  }
}

void PathHandlerNode::PositionCallback(
    const geometry_msgs::PoseStamped & pose_msg) {

  // lastPos = rotatePoseMsgToWorld(lastPos); // 90 deg fix
  listener_.transformPose("world", ros::Time(0), pose_msg, "local_origin", lastPos);

  // Check if we are close enough to current goal to get the next part of the path
  if (path.size() > 0 && std::abs(currentGoal.pose.position.x - lastPos.pose.position.x) < 1 
                      && std::abs(currentGoal.pose.position.y - lastPos.pose.position.y) < 1
                      && std::abs(currentGoal.pose.position.z - lastPos.pose.position.z) < 1) {

    // TODO: get yawdiff(yaw1, yaw2)
    double yaw1 = tf::getYaw(currentGoal.pose.orientation);
    double yaw2 = tf::getYaw(lastPos.pose.orientation);
    double yawDiff = std::abs(yaw2 - yaw1);
    yawDiff -= std::floor(yawDiff / (2*M_PI)) * (2*M_PI);
    double maxYawDiff = M_PI/8.0;
    if (yawDiff < maxYawDiff || yawDiff  > 2*M_PI - maxYawDiff){
      // If we are facing the right direction, then pop the first point of the path
      currentGoal = path[0];
      path.erase(path.begin());

      // If we are keeping the same direction and height increase speed
      if (path.size() > std::floor(speed) && hasSameYawAndAltitude(currentGoal.pose, path[std::floor(speed)].pose)) {
        speed = std::max(maxSpeed, speed+0.1);
      }
      else {
        speed = 1.0;
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
