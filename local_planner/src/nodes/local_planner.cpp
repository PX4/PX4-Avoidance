#include "local_planner/local_planner.h"

#include "local_planner/common.h"
#include "local_planner/planner_functions.h"
#include "local_planner/star_planner.h"
#include "local_planner/tree_node.h"

#include <sensor_msgs/image_encodings.h>

namespace avoidance {

LocalPlanner::LocalPlanner() : star_planner_(new StarPlanner()) {}

LocalPlanner::~LocalPlanner() {}

// update UAV pose
void LocalPlanner::setPose(const Eigen::Vector3f& pos,
                           const Eigen::Quaternionf& q) {
  position_ = pos;
  curr_yaw_fcu_frame_deg_ = getYawFromQuaternion(q);
  curr_yaw_histogram_frame_deg_ = -curr_yaw_fcu_frame_deg_ + 90.0f;

  curr_pitch_deg_ = getPitchFromQuaternion(q);
  star_planner_->setPose(position_, curr_yaw_histogram_frame_deg_);

  if (!currently_armed_ && !disable_rise_to_goal_altitude_) {
    take_off_pose_ = position_;
    reach_altitude_ = false;
  }
}

// set parameters changed by dynamic rconfigure
void LocalPlanner::dynamicReconfigureSetParams(
    avoidance::LocalPlannerNodeConfig& config, uint32_t level) {
  histogram_box_.radius_ = static_cast<float>(config.box_radius_);
  cost_params_.goal_cost_param = config.goal_cost_param_;
  cost_params_.heading_cost_param = config.heading_cost_param_;
  cost_params_.smooth_cost_param = config.smooth_cost_param_;
  velocity_around_obstacles_ =
      static_cast<float>(config.velocity_around_obstacles_);
  velocity_far_from_obstacles_ =
      static_cast<float>(config.velocity_far_from_obstacles_);
  reproj_age_ = static_cast<float>(config.reproj_age_);
  velocity_sigmoid_slope_ = static_cast<float>(config.velocity_sigmoid_slope_);
  no_progress_slope_ = static_cast<float>(config.no_progress_slope_);
  min_cloud_size_ = config.min_cloud_size_;
  min_realsense_dist_ = static_cast<float>(config.min_realsense_dist_);
  timeout_critical_ = config.timeout_critical_;
  timeout_termination_ = config.timeout_termination_;
  children_per_node_ = config.children_per_node_;
  n_expanded_nodes_ = config.n_expanded_nodes_;
  smoothing_margin_degrees_ =
      static_cast<float>(config.smoothing_margin_degrees_);

  if (getGoal().z() != config.goal_z_param) {
    auto goal = getGoal();
    goal.z() = config.goal_z_param;
    setGoal(goal);
  }

  use_vel_setpoints_ = config.use_vel_setpoints_;
  adapt_cost_params_ = config.adapt_cost_params_;
  send_obstacles_fcu_ = config.send_obstacles_fcu_;

  star_planner_->dynamicReconfigureSetStarParams(config, level);

  ROS_DEBUG("\033[0;35m[OA] Dynamic reconfigure call \033[0m");
}

void LocalPlanner::setGoal(const Eigen::Vector3f& goal) {
  goal_ = goal;
  ROS_INFO("===== Set Goal ======: [%f, %f, %f].", goal_.x(), goal_.y(),
           goal_.z());
  applyGoal();
}

Eigen::Vector3f LocalPlanner::getGoal() const { return goal_; }

void LocalPlanner::applyGoal() {
  star_planner_->setGoal(goal_);
  goal_dist_incline_.clear();
}

void LocalPlanner::runPlanner() {
  ROS_INFO("\033[1;35m[OA] Planning started, using %i cameras\n \033[0m",
           static_cast<int>(complete_cloud_.size()));

  // calculate Field of View
  z_FOV_idx_.clear();
  calculateFOV(h_FOV_, v_FOV_, z_FOV_idx_, e_FOV_min_, e_FOV_max_,
               curr_yaw_histogram_frame_deg_, curr_pitch_deg_);

  histogram_box_.setBoxLimits(position_, ground_distance_);

  filterPointCloud(final_cloud_, complete_cloud_, min_cloud_size_,
                   histogram_box_, position_, min_realsense_dist_);

  determineStrategy();
}

void LocalPlanner::create2DObstacleRepresentation(const bool send_to_fcu) {
  // construct histogram if it is needed
  // or if it is required by the FCU
  Histogram propagated_histogram = Histogram(2 * ALPHA_RES);
  Histogram new_histogram = Histogram(ALPHA_RES);
  to_fcu_histogram_.setZero();

  propagateHistogram(propagated_histogram, reprojected_points_,
                     reprojected_points_age_, position_);
  generateNewHistogram(new_histogram, final_cloud_, position_);
  combinedHistogram(hist_is_empty_, new_histogram, propagated_histogram,
                    waypoint_outside_FOV_, z_FOV_idx_, e_FOV_min_, e_FOV_max_);
  if (send_to_fcu) {
    compressHistogramElevation(to_fcu_histogram_, new_histogram);
    updateObstacleDistanceMsg(to_fcu_histogram_);
  }
  polar_histogram_ = new_histogram;

  // create 3D points from combined histogram
  reprojectPoints(polar_histogram_);

  // generate histogram image for logging
  generateHistogramImage(polar_histogram_);
}

void LocalPlanner::generateHistogramImage(Histogram& histogram) {
  histogram_image_data_.clear();
  histogram_image_data_.reserve(GRID_LENGTH_E * GRID_LENGTH_Z);

  // fill image data
  for (int e = GRID_LENGTH_E - 1; e >= 0; e--) {
    for (int z = 0; z < GRID_LENGTH_Z; z++) {
      float depth_val =
          255.f * histogram.get_dist(e, z) / histogram_box_.radius_;
      histogram_image_data_.push_back(
          (int)std::max(0.0f, std::min(255.f, depth_val)));
    }
  }
}

void LocalPlanner::determineStrategy() {
  star_planner_->tree_age_++;

  // clear cost image
  cost_image_data_.clear();
  cost_image_data_.resize(3 * GRID_LENGTH_E * GRID_LENGTH_Z, 0);

  if (disable_rise_to_goal_altitude_) {
    reach_altitude_ = true;
  }
  reach_altitude_ = true;

  if (!reach_altitude_) {
    starting_height_ = std::max(goal_.z() - 0.5f, take_off_pose_.z() + 1.0f);
    ROS_INFO("\033[1;35m[OA] Reach height (%f) first: Go fast\n \033[0m",
             starting_height_);
    waypoint_type_ = reachHeight;

    if (position_.z() > starting_height_) {
      reach_altitude_ = true;
      waypoint_type_ = direct;
    }

    if (send_obstacles_fcu_) {
      create2DObstacleRepresentation(true);
    }
  } else {
    waypoint_type_ = tryPath;

    evaluateProgressRate();
    create2DObstacleRepresentation(send_obstacles_fcu_);

    if (!hist_is_empty_) {
      getCostMatrix(polar_histogram_, goal_, position_,
                    curr_yaw_histogram_frame_deg_, last_sent_waypoint_,
                    cost_params_, velocity_.norm() < 0.1f,
                    smoothing_margin_degrees_, cost_matrix_, cost_image_data_);

      star_planner_->setParams(cost_params_);
      star_planner_->setFOV(h_FOV_, v_FOV_);
      star_planner_->setReprojectedPoints(reprojected_points_,
                                          reprojected_points_age_);

      // set last chosen direction for smoothing
      PolarPoint last_wp_pol = cartesianToPolar(last_sent_waypoint_, position_);
      last_wp_pol.r = (position_ - goal_).norm();
      Eigen::Vector3f projected_last_wp =
          polarToCartesian(last_wp_pol, position_);
      star_planner_->setLastDirection(projected_last_wp);

      // build search tree
      star_planner_->buildLookAheadTree();

      last_path_time_ = ros::Time::now();
    }
  }
  position_old_ = position_;
}

void LocalPlanner::updateObstacleDistanceMsg(Histogram hist) {
  sensor_msgs::LaserScan msg = {};
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "local_origin";
  msg.angle_increment = static_cast<double>(ALPHA_RES) * M_PI / 180.0;
  msg.range_min = 0.2f;
  msg.range_max = 20.0f;

  // turn idxs 180 degress to point to local north instead of south
  std::vector<int> z_FOV_idx_north;
  z_FOV_idx_north.reserve(z_FOV_idx_.size());

  for (size_t i = 0; i < z_FOV_idx_.size(); i++) {
    int new_idx = z_FOV_idx_[i] + GRID_LENGTH_Z / 2;

    if (new_idx >= GRID_LENGTH_Z) {
      new_idx = new_idx - GRID_LENGTH_Z;
    }

    z_FOV_idx_north.push_back(new_idx);
  }

  msg.ranges.reserve(GRID_LENGTH_Z);
  for (int idx = 0; idx < GRID_LENGTH_Z; idx++) {
    float range;

    if (std::find(z_FOV_idx_north.begin(), z_FOV_idx_north.end(), idx) ==
        z_FOV_idx_north.end()) {
      range = UINT16_MAX;
    } else {
      int hist_idx = idx - GRID_LENGTH_Z / 2;

      if (hist_idx < 0) {
        hist_idx = hist_idx + GRID_LENGTH_Z;
      }

      if (hist.get_dist(0, hist_idx) == 0.0f) {
        range = msg.range_max + 1.0f;
      } else {
        range = hist.get_dist(0, hist_idx);
      }
    }

    msg.ranges.push_back(range);
  }

  distance_data_ = msg;
}

void LocalPlanner::updateObstacleDistanceMsg() {
  sensor_msgs::LaserScan msg = {};
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "local_origin";
  msg.angle_increment = static_cast<double>(ALPHA_RES) * M_PI / 180.0;
  msg.range_min = 0.2f;
  msg.range_max = 20.0f;

  distance_data_ = msg;
}

// get 3D points from old histogram
void LocalPlanner::reprojectPoints(Histogram histogram) {
  float dist;
  int age;
  // ALPHA_RES%2=0 as per definition, see histogram.h
  int half_res = ALPHA_RES / 2;
  Eigen::Vector3f temp_array[4];

  std::array<PolarPoint, 4> p_pol;
  reprojected_points_age_.clear();

  reprojected_points_.points.clear();
  reprojected_points_.header.stamp = final_cloud_.header.stamp;
  reprojected_points_.header.frame_id = "local_origin";

  for (int e = 0; e < GRID_LENGTH_E; e++) {
    for (int z = 0; z < GRID_LENGTH_Z; z++) {
      if (histogram.get_dist(e, z) > FLT_MIN) {
        for (auto& i : p_pol) {
          i.r = histogram.get_dist(e, z);
          i = histogramIndexToPolar(e, z, ALPHA_RES, i.r);
        }
        // transform from array index to angle
        p_pol[0].e += half_res;
        p_pol[0].z += half_res;
        p_pol[1].e -= half_res;
        p_pol[1].z += half_res;
        p_pol[2].e += half_res;
        p_pol[2].z -= half_res;
        p_pol[3].e -= half_res;
        p_pol[3].z -= half_res;

        // transform from Polar to Cartesian
        temp_array[0] = polarToCartesian(p_pol[0], position_);
        temp_array[1] = polarToCartesian(p_pol[1], position_);
        temp_array[2] = polarToCartesian(p_pol[2], position_);
        temp_array[3] = polarToCartesian(p_pol[3], position_);

        for (int i = 0; i < 4; i++) {
          dist = (position_ - temp_array[i]).norm();
          age = histogram.get_age(e, z);

          if (dist < 2.0f * histogram_box_.radius_ && dist > 0.3f &&
              age < reproj_age_) {
            reprojected_points_.points.push_back(toXYZ(temp_array[i]));
            reprojected_points_age_.push_back(age);
          }
        }
      }
    }
  }
}

// calculate the correct weight between fly over and fly around
void LocalPlanner::evaluateProgressRate() {
  if (reach_altitude_ && adapt_cost_params_) {
    float goal_dist = (position_ - goal_).norm();
    float goal_dist_old = (position_old_ - goal_).norm();

    ros::Time time = ros::Time::now();
    float time_diff_sec =
        static_cast<float>((time - integral_time_old_).toSec());
    float incline = (goal_dist - goal_dist_old) / time_diff_sec;
    integral_time_old_ = time;

    goal_dist_incline_.push_back(incline);
    if (goal_dist_incline_.size() > dist_incline_window_size_) {
      goal_dist_incline_.pop_front();
    }

    float sum_incline = 0.0f;
    int n_incline = 0;
    for (size_t i = 0; i < goal_dist_incline_.size(); i++) {
      sum_incline += goal_dist_incline_[i];
      n_incline++;
    }
    float avg_incline = sum_incline / static_cast<float>(n_incline);

    if (avg_incline > no_progress_slope_ &&
        goal_dist_incline_.size() == dist_incline_window_size_) {
      if (cost_params_.height_change_cost_param_adapted > 0.75f) {
        cost_params_.height_change_cost_param_adapted -= 0.02f;
      }
    }
    if (avg_incline < no_progress_slope_) {
      if (cost_params_.height_change_cost_param_adapted <
          cost_params_.height_change_cost_param - 0.03f) {
        cost_params_.height_change_cost_param_adapted += 0.03f;
      }
    }
    ROS_DEBUG(
        "\033[0;35m[OA] Progress rate to goal: %f, adapted height change cost: "
        "%f .\033[0m",
        avg_incline, cost_params_.height_change_cost_param_adapted);
  } else {
    cost_params_.height_change_cost_param_adapted =
        cost_params_.height_change_cost_param;
  }
}

Eigen::Vector3f LocalPlanner::getPosition() const { return position_; }

const pcl::PointCloud<pcl::PointXYZ>& LocalPlanner::getCroppedCloud() const {
  return final_cloud_;
}

const pcl::PointCloud<pcl::PointXYZ>& LocalPlanner::getReprojectedPoints()
    const {
  return reprojected_points_;
}

void LocalPlanner::setCurrentVelocity(const Eigen::Vector3f& vel) {
  velocity_ = vel;
}

void LocalPlanner::getTree(
    std::vector<TreeNode>& tree, std::vector<int>& closed_set,
    std::vector<Eigen::Vector3f>& path_node_positions) const {
  tree = star_planner_->tree_;
  closed_set = star_planner_->closed_set_;
  path_node_positions = star_planner_->path_node_positions_;
}

void LocalPlanner::getObstacleDistanceData(
    sensor_msgs::LaserScan& obstacle_distance) {
  obstacle_distance = distance_data_;
}

avoidanceOutput LocalPlanner::getAvoidanceOutput() const {
  avoidanceOutput out;
  out.waypoint_type = waypoint_type_;

  out.obstacle_ahead = !hist_is_empty_;
  out.velocity_around_obstacles = velocity_around_obstacles_;
  out.velocity_far_from_obstacles = velocity_far_from_obstacles_;
  out.last_path_time = last_path_time_;

  out.take_off_pose = take_off_pose_;

  out.path_node_positions = star_planner_->path_node_positions_;
  return out;
}
}
