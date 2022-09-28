#include "local_planner/local_planner_visualization.h"

#include "avoidance/common.h"
#include "local_planner/planner_functions.h"
#include "local_planner/tree_node.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace avoidance {

// initialize subscribers for local planner visualization topics
void LocalPlannerVisualization::initializePublishers(ros::NodeHandle& nh) {
  local_pointcloud_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("local_pointcloud", 1);
  pointcloud_size_pub_ = nh.advertise<std_msgs::UInt32>("pointcloud_size", 1);
  bounding_box_pub_ = nh.advertise<visualization_msgs::MarkerArray>("bounding_box", 1);
  ground_measurement_pub_ = nh.advertise<visualization_msgs::Marker>("ground_measurement", 1);
  original_wp_pub_ = nh.advertise<visualization_msgs::Marker>("original_waypoint", 1);
  adapted_wp_pub_ = nh.advertise<visualization_msgs::Marker>("adapted_waypoint", 1);
  smoothed_wp_pub_ = nh.advertise<visualization_msgs::Marker>("smoothed_waypoint", 1);
  complete_tree_pub_ = nh.advertise<visualization_msgs::Marker>("complete_tree", 1);
  tree_path_pub_ = nh.advertise<visualization_msgs::Marker>("tree_path", 1);
  marker_goal_pub_ = nh.advertise<visualization_msgs::MarkerArray>("goal_position", 1);
  path_actual_pub_ = nh.advertise<visualization_msgs::Marker>("path_actual", 1);
  path_waypoint_pub_ = nh.advertise<visualization_msgs::Marker>("path_waypoint", 1);
  path_adapted_waypoint_pub_ = nh.advertise<visualization_msgs::Marker>("path_adapted_waypoint", 1);
  current_waypoint_pub_ = nh.advertise<visualization_msgs::Marker>("current_setpoint", 1);
  histogram_image_pub_ = nh.advertise<sensor_msgs::Image>("histogram_image", 1);
  cost_image_pub_ = nh.advertise<sensor_msgs::Image>("cost_image", 1);
  closest_point_pub_ = nh.advertise<visualization_msgs::Marker>("closest_point", 1);
  deg60_point_pub_ = nh.advertise<visualization_msgs::Marker>("deg60_point", 1);
  fov_pub_ = nh.advertise<visualization_msgs::Marker>("fov", 4);
  range_scan_pub_ = nh.advertise<visualization_msgs::Marker>("range_scan", 1);
}

void LocalPlannerVisualization::visualizePlannerData(const LocalPlanner& planner,
                                                     const Eigen::Vector3f& newest_waypoint_position,
                                                     const Eigen::Vector3f& newest_adapted_waypoint_position,
                                                     const Eigen::Vector3f& newest_position,
                                                     const Eigen::Quaternionf& newest_orientation) const {
  // visualize clouds
  local_pointcloud_pub_.publish(planner.getPointcloud());
  std_msgs::UInt32 msg;
  msg.data = static_cast<uint32_t>(planner.getPointcloud().size());
  pointcloud_size_pub_.publish(msg);

  // visualize tree calculation
  std::vector<TreeNode> tree;
  std::vector<int> closed_set;
  std::vector<Eigen::Vector3f> path_node_positions;
  planner.getTree(tree, closed_set, path_node_positions);
  publishTree(tree, closed_set, path_node_positions);

  // visualize goal
  publishGoal(toPoint(planner.getGoal()));

  // publish histogram image
  publishDataImages(planner.histogram_image_data_, planner.cost_image_data_, newest_waypoint_position,
                    newest_adapted_waypoint_position, newest_position, newest_orientation);

  // publish the FOV
  publishFOV(planner.getFOV(), planner.getSensorRange());

  // range scan
  publishRangeScan(planner.distance_data_, newest_position);
}

void LocalPlannerVisualization::publishFOV(const std::vector<FOV>& fov_vec, float max_range) const {
  Eigen::Vector3f drone_pos = Eigen::Vector3f(0.f, 0.f, 0.f);
  for (int i = 0; i < fov_vec.size(); ++i) {
    PolarPoint p1(fov_vec[i].pitch_deg - fov_vec[i].v_fov_deg / 2.f, fov_vec[i].yaw_deg + fov_vec[i].h_fov_deg / 2.f,
                  max_range);
    PolarPoint p2(fov_vec[i].pitch_deg + fov_vec[i].v_fov_deg / 2.f, fov_vec[i].yaw_deg + fov_vec[i].h_fov_deg / 2.f,
                  max_range);
    PolarPoint p3(fov_vec[i].pitch_deg + fov_vec[i].v_fov_deg / 2.f, fov_vec[i].yaw_deg - fov_vec[i].h_fov_deg / 2.f,
                  max_range);
    PolarPoint p4(fov_vec[i].pitch_deg - fov_vec[i].v_fov_deg / 2.f, fov_vec[i].yaw_deg - fov_vec[i].h_fov_deg / 2.f,
                  max_range);

    visualization_msgs::Marker m;
    m.header.frame_id = "fcu";
    m.header.stamp = ros::Time::now();
    m.id = i;
    m.type = visualization_msgs::Marker::TRIANGLE_LIST;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = 1.0;
    m.scale.y = 1.0;
    m.scale.z = 1.0;
    m.color.a = 0.4;
    m.color.r = 0.5;
    m.color.g = 0.5;
    m.color.b = 1.0;

    // side 1
    m.points.push_back(toPoint(drone_pos));
    m.points.push_back(toPoint(polarFCUToCartesian(p1, drone_pos)));
    m.points.push_back(toPoint(polarFCUToCartesian(p2, drone_pos)));
    // side 2
    m.points.push_back(toPoint(drone_pos));
    m.points.push_back(toPoint(polarFCUToCartesian(p2, drone_pos)));
    m.points.push_back(toPoint(polarFCUToCartesian(p3, drone_pos)));
    // side 3
    m.points.push_back(toPoint(drone_pos));
    m.points.push_back(toPoint(polarFCUToCartesian(p3, drone_pos)));
    m.points.push_back(toPoint(polarFCUToCartesian(p4, drone_pos)));
    // side 4
    m.points.push_back(toPoint(drone_pos));
    m.points.push_back(toPoint(polarFCUToCartesian(p4, drone_pos)));
    m.points.push_back(toPoint(polarFCUToCartesian(p1, drone_pos)));

    fov_pub_.publish(m);
  }
}

void LocalPlannerVisualization::publishRangeScan(const sensor_msgs::LaserScan& scan,
                                                 const Eigen::Vector3f& newest_position) const {
  visualization_msgs::Marker m;
  m.header.frame_id = "local_origin";
  m.header.stamp = ros::Time::now();
  m.id = 0;
  m.type = visualization_msgs::Marker::TRIANGLE_LIST;
  m.action = visualization_msgs::Marker::ADD;
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.a = 0.7;
  m.color.r = 1.0;
  m.color.g = 1.0;
  m.color.b = 1.0;

  std_msgs::ColorRGBA c;
  c.a = 0.7;

  for (int i = 0; i < scan.ranges.size(); ++i) {
    PolarPoint p1(0, RAD_TO_DEG * (i + 0.5) * scan.angle_increment, std::min(scan.range_max, scan.ranges[i]));
    PolarPoint p2(0, RAD_TO_DEG * (i - 0.5) * scan.angle_increment, std::min(scan.range_max, scan.ranges[i]));

    if (std::isnan(scan.ranges[i])) {
      c.r = 1.0;
      c.g = 0.0;
      c.b = 0.0;
    } else if (scan.ranges[i] > scan.range_max) {
      c.r = 0.0;
      c.g = 1.0;
      c.b = 0.0;

    } else {
      c.g = scan.ranges[i] / scan.range_max;
      c.r = 1.0 - scan.ranges[i] / scan.range_max;
      c.b = 0.0;
    }
    m.colors.push_back(c);
    m.colors.push_back(c);
    m.colors.push_back(c);

    // side 1
    m.points.push_back(toPoint(newest_position));
    m.points.push_back(toPoint(polarHistogramToCartesian(p1, newest_position)));
    m.points.push_back(toPoint(polarHistogramToCartesian(p2, newest_position)));
  }

  range_scan_pub_.publish(m);
}

void LocalPlannerVisualization::publishOfftrackPoints(Eigen::Vector3f& closest_pt, Eigen::Vector3f& deg60_pt) {
  visualization_msgs::Marker m;

  m.header.frame_id = "local_origin";
  m.header.stamp = ros::Time::now();
  m.type = visualization_msgs::Marker::SPHERE;
  m.action = visualization_msgs::Marker::ADD;
  m.scale.x = 0.2;
  m.scale.y = 0.2;
  m.scale.z = 0.2;
  m.color.a = 1.0;
  m.color.r = 1.0;
  m.color.g = 0.0;
  m.color.b = 0.0;
  m.lifetime = ros::Duration();
  m.id = 0;
  m.pose.position.x = closest_pt.x();
  m.pose.position.y = closest_pt.y();
  m.pose.position.z = closest_pt.z();
  closest_point_pub_.publish(m);

  m.color.r = 0.0;
  m.color.g = 0.0;
  m.color.b = 1.0;
  m.pose.position.x = deg60_pt.x();
  m.pose.position.y = deg60_pt.y();
  m.pose.position.z = deg60_pt.z();
  deg60_point_pub_.publish(m);
}

void LocalPlannerVisualization::publishTree(const std::vector<TreeNode>& tree, const std::vector<int>& closed_set,
                                            const std::vector<Eigen::Vector3f>& path_node_positions) const {
  visualization_msgs::Marker tree_marker;
  tree_marker.header.frame_id = "local_origin";
  tree_marker.header.stamp = ros::Time::now();
  tree_marker.id = 0;
  tree_marker.type = visualization_msgs::Marker::LINE_LIST;
  tree_marker.action = visualization_msgs::Marker::ADD;
  tree_marker.pose.orientation.w = 1.0;
  tree_marker.scale.x = 0.05;
  tree_marker.color.a = 0.8;
  tree_marker.color.r = 0.4;
  tree_marker.color.g = 0.0;
  tree_marker.color.b = 0.6;

  visualization_msgs::Marker path_marker;
  path_marker.header.frame_id = "local_origin";
  path_marker.header.stamp = ros::Time::now();
  path_marker.id = 0;
  path_marker.type = visualization_msgs::Marker::LINE_LIST;
  path_marker.action = visualization_msgs::Marker::ADD;
  path_marker.pose.orientation.w = 1.0;
  path_marker.scale.x = 0.05;
  path_marker.color.a = 0.8;
  path_marker.color.r = 1.0;
  path_marker.color.g = 0.0;
  path_marker.color.b = 0.0;

  tree_marker.points.reserve(closed_set.size() * 2);
  for (size_t i = 0; i < closed_set.size(); i++) {
    int node_nr = closed_set[i];
    geometry_msgs::Point p1 = toPoint(tree[node_nr].getPosition());
    int origin = tree[node_nr].origin_;
    geometry_msgs::Point p2 = toPoint(tree[origin].getPosition());
    tree_marker.points.push_back(p1);
    tree_marker.points.push_back(p2);
  }

  path_marker.points.reserve(path_node_positions.size() * 2);
  for (size_t i = 1; i < path_node_positions.size(); i++) {
    path_marker.points.push_back(toPoint(path_node_positions[i - 1]));
    path_marker.points.push_back(toPoint(path_node_positions[i]));
  }

  complete_tree_pub_.publish(tree_marker);
  tree_path_pub_.publish(path_marker);
}

void LocalPlannerVisualization::publishGoal(const geometry_msgs::Point& goal) const {
  visualization_msgs::MarkerArray marker_goal;
  visualization_msgs::Marker m;

  m.header.frame_id = "local_origin";
  m.header.stamp = ros::Time::now();
  m.type = visualization_msgs::Marker::SPHERE;
  m.action = visualization_msgs::Marker::ADD;
  m.scale.x = 0.5;
  m.scale.y = 0.5;
  m.scale.z = 0.5;
  m.color.a = 1.0;
  m.color.r = 1.0;
  m.color.g = 1.0;
  m.color.b = 0.0;
  m.lifetime = ros::Duration();
  m.id = 0;
  m.pose.position = goal;
  marker_goal.markers.push_back(m);
  marker_goal_pub_.publish(marker_goal);
}

void LocalPlannerVisualization::publishDataImages(const std::vector<uint8_t>& histogram_image_data,
                                                  const std::vector<uint8_t>& cost_image_data,
                                                  const Eigen::Vector3f& newest_waypoint_position,
                                                  const Eigen::Vector3f& newest_adapted_waypoint_position,
                                                  const Eigen::Vector3f& newest_position,
                                                  const Eigen::Quaternionf newest_orientation) const {
  sensor_msgs::Image cost_img;
  cost_img.header.stamp = ros::Time::now();
  cost_img.height = GRID_LENGTH_E;
  cost_img.width = GRID_LENGTH_Z;
  cost_img.encoding = "rgb8";
  cost_img.is_bigendian = 0;
  cost_img.step = 3 * cost_img.width;
  cost_img.data = cost_image_data;

  // current orientation
  float curr_yaw_fcu_frame = getYawFromQuaternion(newest_orientation);
  float yaw_angle_histogram_frame = -static_cast<float>(curr_yaw_fcu_frame) * 180.0f / M_PI_F + 90.0f;
  PolarPoint heading_pol(0, yaw_angle_histogram_frame, 1.0);
  Eigen::Vector2i heading_index = polarToHistogramIndex(heading_pol, ALPHA_RES);

  // current setpoint
  PolarPoint waypoint_pol = cartesianToPolarHistogram(newest_waypoint_position, newest_position);
  Eigen::Vector2i waypoint_index = polarToHistogramIndex(waypoint_pol, ALPHA_RES);
  PolarPoint adapted_waypoint_pol = cartesianToPolarHistogram(newest_adapted_waypoint_position, newest_position);
  Eigen::Vector2i adapted_waypoint_index = polarToHistogramIndex(adapted_waypoint_pol, ALPHA_RES);

  // color in the image
  if (cost_img.data.size() == 3 * GRID_LENGTH_E * GRID_LENGTH_Z) {
    // current heading blue
    cost_img.data[colorImageIndex(heading_index.y(), heading_index.x(), 2)] = 255.f;

    // waypoint white
    cost_img.data[colorImageIndex(waypoint_index.y(), waypoint_index.x(), 0)] = 255.f;
    cost_img.data[colorImageIndex(waypoint_index.y(), waypoint_index.x(), 1)] = 255.f;
    cost_img.data[colorImageIndex(waypoint_index.y(), waypoint_index.x(), 2)] = 255.f;

    // adapted waypoint light blue
    cost_img.data[colorImageIndex(adapted_waypoint_index.y(), adapted_waypoint_index.x(), 1)] = 255.f;
    cost_img.data[colorImageIndex(adapted_waypoint_index.y(), adapted_waypoint_index.x(), 2)] = 255.f;
  }

  // histogram image
  sensor_msgs::Image hist_img;
  hist_img.header.stamp = ros::Time::now();
  hist_img.height = GRID_LENGTH_E;
  hist_img.width = GRID_LENGTH_Z;
  hist_img.encoding = sensor_msgs::image_encodings::MONO8;
  hist_img.is_bigendian = 0;
  hist_img.step = 255;
  hist_img.data = histogram_image_data;

  histogram_image_pub_.publish(hist_img);
  cost_image_pub_.publish(cost_img);
}

void LocalPlannerVisualization::visualizeWaypoints(const Eigen::Vector3f& goto_position,
                                                   const Eigen::Vector3f& adapted_goto_position,
                                                   const Eigen::Vector3f& smoothed_goto_position) const {
  visualization_msgs::Marker sphere1;
  visualization_msgs::Marker sphere2;
  visualization_msgs::Marker sphere3;

  ros::Time now = ros::Time::now();

  sphere1.header.frame_id = "local_origin";
  sphere1.header.stamp = now;
  sphere1.id = 0;
  sphere1.type = visualization_msgs::Marker::SPHERE;
  sphere1.action = visualization_msgs::Marker::ADD;
  sphere1.pose.position = toPoint(goto_position);
  sphere1.pose.orientation.x = 0.0;
  sphere1.pose.orientation.y = 0.0;
  sphere1.pose.orientation.z = 0.0;
  sphere1.pose.orientation.w = 1.0;
  sphere1.scale.x = 0.2;
  sphere1.scale.y = 0.2;
  sphere1.scale.z = 0.2;
  sphere1.color.a = 0.8;
  sphere1.color.r = 0.5;
  sphere1.color.g = 1.0;
  sphere1.color.b = 0.0;

  sphere2.header.frame_id = "local_origin";
  sphere2.header.stamp = now;
  sphere2.id = 0;
  sphere2.type = visualization_msgs::Marker::SPHERE;
  sphere2.action = visualization_msgs::Marker::ADD;
  sphere2.pose.position = toPoint(adapted_goto_position);
  sphere2.pose.orientation.x = 0.0;
  sphere2.pose.orientation.y = 0.0;
  sphere2.pose.orientation.z = 0.0;
  sphere2.pose.orientation.w = 1.0;
  sphere2.scale.x = 0.2;
  sphere2.scale.y = 0.2;
  sphere2.scale.z = 0.2;
  sphere2.color.a = 0.8;
  sphere2.color.r = 1.0;
  sphere2.color.g = 1.0;
  sphere2.color.b = 0.0;

  sphere3.header.frame_id = "local_origin";
  sphere3.header.stamp = now;
  sphere3.id = 0;
  sphere3.type = visualization_msgs::Marker::SPHERE;
  sphere3.action = visualization_msgs::Marker::ADD;
  sphere3.pose.position = toPoint(smoothed_goto_position);
  sphere3.pose.orientation.x = 0.0;
  sphere3.pose.orientation.y = 0.0;
  sphere3.pose.orientation.z = 0.0;
  sphere3.pose.orientation.w = 1.0;
  sphere3.scale.x = 0.2;
  sphere3.scale.y = 0.2;
  sphere3.scale.z = 0.2;
  sphere3.color.a = 0.8;
  sphere3.color.r = 1.0;
  sphere3.color.g = 0.5;
  sphere3.color.b = 0.0;

  original_wp_pub_.publish(sphere1);
  adapted_wp_pub_.publish(sphere2);
  smoothed_wp_pub_.publish(sphere3);
}

void LocalPlannerVisualization::publishPaths(const Eigen::Vector3f& last_position,
                                             const Eigen::Vector3f& newest_position, const Eigen::Vector3f& last_wp,
                                             const Eigen::Vector3f& newest_wp, const Eigen::Vector3f& last_adapted_wp,
                                             const Eigen::Vector3f& newest_adapted_wp) {
  // publish actual path
  visualization_msgs::Marker path_actual_marker;
  path_actual_marker.header.frame_id = "local_origin";
  path_actual_marker.header.stamp = ros::Time::now();
  path_actual_marker.id = path_length_;
  path_actual_marker.type = visualization_msgs::Marker::LINE_STRIP;
  path_actual_marker.action = visualization_msgs::Marker::ADD;
  path_actual_marker.pose.orientation.w = 1.0;
  path_actual_marker.scale.x = 0.03;
  path_actual_marker.color.a = 1.0;
  path_actual_marker.color.r = 0.0;
  path_actual_marker.color.g = 1.0;
  path_actual_marker.color.b = 0.0;

  path_actual_marker.points.push_back(toPoint(last_position));
  path_actual_marker.points.push_back(toPoint(newest_position));
  path_actual_pub_.publish(path_actual_marker);

  // publish path set by calculated waypoints
  visualization_msgs::Marker path_waypoint_marker;
  path_waypoint_marker.header.frame_id = "local_origin";
  path_waypoint_marker.header.stamp = ros::Time::now();
  path_waypoint_marker.id = path_length_;
  path_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
  path_waypoint_marker.action = visualization_msgs::Marker::ADD;
  path_waypoint_marker.pose.orientation.w = 1.0;
  path_waypoint_marker.scale.x = 0.02;
  path_waypoint_marker.color.a = 1.0;
  path_waypoint_marker.color.r = 1.0;
  path_waypoint_marker.color.g = 0.0;
  path_waypoint_marker.color.b = 0.0;

  path_waypoint_marker.points.push_back(toPoint(last_wp));
  path_waypoint_marker.points.push_back(toPoint(newest_wp));
  path_waypoint_pub_.publish(path_waypoint_marker);

  // publish path set by calculated waypoints
  visualization_msgs::Marker path_adapted_waypoint_marker;
  path_adapted_waypoint_marker.header.frame_id = "local_origin";
  path_adapted_waypoint_marker.header.stamp = ros::Time::now();
  path_adapted_waypoint_marker.id = path_length_;
  path_adapted_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
  path_adapted_waypoint_marker.action = visualization_msgs::Marker::ADD;
  path_adapted_waypoint_marker.pose.orientation.w = 1.0;
  path_adapted_waypoint_marker.scale.x = 0.02;
  path_adapted_waypoint_marker.color.a = 1.0;
  path_adapted_waypoint_marker.color.r = 0.0;
  path_adapted_waypoint_marker.color.g = 0.0;
  path_adapted_waypoint_marker.color.b = 1.0;

  path_adapted_waypoint_marker.points.push_back(toPoint(last_adapted_wp));
  path_adapted_waypoint_marker.points.push_back(toPoint(newest_adapted_wp));
  path_adapted_waypoint_pub_.publish(path_adapted_waypoint_marker);

  path_length_++;
}

void LocalPlannerVisualization::publishCurrentSetpoint(const geometry_msgs::Twist& wp,
                                                       const PlannerState& waypoint_type,
                                                       const Eigen::Vector3f& newest_position) const {
  visualization_msgs::Marker setpoint;
  setpoint.header.frame_id = "local_origin";
  setpoint.header.stamp = ros::Time::now();
  setpoint.id = 0;
  setpoint.type = visualization_msgs::Marker::ARROW;
  setpoint.action = visualization_msgs::Marker::ADD;

  geometry_msgs::Point tip;
  geometry_msgs::Point newest_pos;

  newest_pos.x = newest_position(0);
  newest_pos.y = newest_position(1);
  newest_pos.z = newest_position(2);
  tip.x = newest_pos.x + wp.linear.x;
  tip.y = newest_pos.y + wp.linear.y;
  tip.z = newest_pos.z + wp.linear.z;
  setpoint.points.push_back(newest_pos);
  setpoint.points.push_back(tip);
  setpoint.scale.x = 0.1;
  setpoint.scale.y = 0.1;
  setpoint.scale.z = 0.1;
  setpoint.color.a = 1.0;

  switch (waypoint_type) {
    case PlannerState::LOITER: {
      setpoint.color.r = 1.0;
      setpoint.color.g = 1.0;
      setpoint.color.b = 0.0;
      break;
    }
    case PlannerState::TRY_PATH: {
      setpoint.color.r = 0.0;
      setpoint.color.g = 1.0;
      setpoint.color.b = 0.0;
      break;
    }
    case PlannerState::DIRECT: {
      setpoint.color.r = 0.0;
      setpoint.color.g = 0.0;
      setpoint.color.b = 1.0;
      break;
    }
    case PlannerState::ALTITUDE_CHANGE: {
      setpoint.color.r = 1.0;
      setpoint.color.g = 0.0;
      setpoint.color.b = 1.0;
      break;
    }
  }

  current_waypoint_pub_.publish(setpoint);
}
}
