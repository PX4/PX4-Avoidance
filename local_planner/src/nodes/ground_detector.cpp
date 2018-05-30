#include "ground_detector.h"

GroundDetector::GroundDetector() {}

GroundDetector::GroundDetector(const GroundDetector &detector)
    : min_dist_to_ground_{detector.min_dist_to_ground_},
      ground_heights_{detector.ground_heights_},
      ground_xmax_{detector.ground_xmax_},
      ground_xmin_{detector.ground_xmin_},
      ground_ymax_{detector.ground_ymax_},
      ground_ymin_{detector.ground_ymin_} {}

GroundDetector::~GroundDetector() {}

// set parameters changed by dynamic reconfigure
void GroundDetector::dynamicReconfigureSetGroundParams(
    avoidance::LocalPlannerNodeConfig &config, uint32_t level) {
  ground_inlier_angle_threshold_ = config.ground_inlier_angle_threshold_;
  ground_inlier_distance_threshold_ = config.ground_inlier_distance_threshold_;
  min_cloud_size_ = config.min_cloud_size_;
  min_plane_points_ = config.min_plane_points_;
  min_plane_percentage_ = config.min_plane_percentage_;
  min_dist_to_ground_ = config.min_dist_to_ground_;
  ground_box_size_.zmin_ = 1.5 * min_dist_to_ground_;
  ground_box_size_.xmin_ =
      1.5 * (ground_box_size_.zmin_ / tan((V_FOV / 2.0) * M_PI / 180));
  ground_box_size_.xmax_ =
      1.5 * (ground_box_size_.zmin_ / tan((V_FOV / 2.0) * M_PI / 180));
  ground_box_size_.ymin_ =
      1.5 * (ground_box_size_.zmin_ / tan((V_FOV / 2.0) * M_PI / 180));
  ground_box_size_.ymax_ =
      1.5 * (ground_box_size_.zmin_ / tan((V_FOV / 2.0) * M_PI / 180));
}

void GroundDetector::reset() {
  ground_heights_.clear();
  ground_xmax_.clear();
  ground_xmin_.clear();
  ground_ymax_.clear();
  ground_ymin_.clear();
}

void GroundDetector::getHeightInformation(bool &over_obstacle, bool &too_low,
                                          bool &is_near_min_height) {
  over_obstacle = over_obstacle_;
  too_low = too_low_;
  is_near_min_height = is_near_min_height_;
}

double GroundDetector::getMargin() { return begin_rise_; }

void GroundDetector::getGroundCloudForVisualization(
    pcl::PointCloud<pcl::PointXYZ> &ground_cloud) {
  ground_cloud = ground_cloud_;
}

void GroundDetector::getGroundDataForVisualization(
    geometry_msgs::Point &closest_point_on_ground,
    geometry_msgs::Quaternion &ground_orientation,
    std::vector<double> &ground_heights, std::vector<double> &ground_xmax,
    std::vector<double> &ground_xmin, std::vector<double> &ground_ymax,
    std::vector<double> &ground_ymin) {
  closest_point_on_ground = closest_point_on_ground_;
  ground_orientation = ground_orientation_;
  ground_heights = ground_heights_;
  ground_xmax = ground_xmax_;
  ground_xmin = ground_xmin_;
  ground_ymax = ground_ymax_;
  ground_ymin = ground_ymin_;
}

// Print Internal Height Map
void GroundDetector::logData(std::string log_name) {
  std::ofstream myfile2(("InternalHeightMap_" + log_name).c_str(),
                        std::ofstream::app);
  myfile2 << pose_.header.stamp.sec << "\t" << pose_.header.stamp.nsec << "\t"
          << 0 << "\t" << 0 << "\t" << 0 << "\n";
  int i = 0;
  for (std::vector<double>::iterator it = ground_heights_.begin();
       it != ground_heights_.end(); ++it) {
    myfile2 << ground_heights_[i] << "\t" << ground_xmin_[i] << "\t"
            << ground_xmax_[i] << "\t" << ground_ymin_[i] << "\t"
            << ground_ymax_[i] << "\n";
    i++;
  }
  myfile2.close();
}

void GroundDetector::setParams(double min_dist_to_ground,
                               double min_cloud_size) {
  min_cloud_size_ = min_cloud_size;
  min_dist_to_ground_ = min_dist_to_ground;
}

void GroundDetector::initializeGroundBox(double min_dist_to_ground) {
  min_dist_to_ground_ = min_dist_to_ground;
  ground_box_size_.zmin_ = 1.5 * min_dist_to_ground;
  ground_box_size_.xmin_ =
      1.5 * (ground_box_size_.zmin_ / tan((V_FOV / 2.0) * M_PI / 180));
  ground_box_size_.xmax_ =
      1.5 * (ground_box_size_.zmin_ / tan((V_FOV / 2.0) * M_PI / 180));
  ground_box_size_.ymin_ =
      1.5 * (ground_box_size_.zmin_ / tan((V_FOV / 2.0) * M_PI / 180));
  ground_box_size_.ymax_ =
      1.5 * (ground_box_size_.zmin_ / tan((V_FOV / 2.0) * M_PI / 180));
}

void GroundDetector::setPose(geometry_msgs::PoseStamped pose) { pose_ = pose; }

// crop cloud to groundbox and estimate ground
void GroundDetector::detectGround(
    pcl::PointCloud<pcl::PointXYZ> &complete_cloud) {
  std::clock_t start_time = std::clock();
  pcl::PointCloud<pcl::PointXYZ>::iterator pcl_it;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(
      new pcl::PointCloud<pcl::PointXYZ>);
  ground_cloud_.points.clear();
  float distance;

  for (pcl_it = complete_cloud.begin(); pcl_it != complete_cloud.end();
       ++pcl_it) {
    // Check if the point is invalid
    if (!std::isnan(pcl_it->x) && !std::isnan(pcl_it->y) &&
        !std::isnan(pcl_it->z)) {
      if (ground_box_.isPointWithin(pcl_it->x, pcl_it->y, pcl_it->z)) {
        cloud_temp->points.push_back(
            pcl::PointXYZ(pcl_it->x, pcl_it->y, pcl_it->z));
      }
    }
  }

  ground_cloud_.header.stamp = complete_cloud.header.stamp;
  ground_cloud_.header.frame_id = complete_cloud.header.frame_id;
  ground_cloud_.width = cloud_temp->points.size();
  ground_cloud_.height = 1;
  ground_cloud_.points = cloud_temp->points;

  // fit horizontal plane for ground estimation
  fitPlane();
  ROS_DEBUG("Ground detection took %2.2fms. Ground detected: %d.",
            (std::clock() - start_time) / (double)(CLOCKS_PER_SEC / 1000),
            ground_detected_);
}

// fit plane through groud cloud
void GroundDetector::fitPlane() {
  std::clock_t start_time = std::clock();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  cloud->width = ground_cloud_.width;
  cloud->height = ground_cloud_.height;
  cloud->points = ground_cloud_.points;

  if (ground_cloud_.width > min_cloud_size_) {
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);

    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(ground_inlier_distance_threshold_);
    seg.setInputCloud(cloud);
    seg.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0));
    seg.setEpsAngle(ground_inlier_angle_threshold_ * M_PI / 180.0);
    seg.segment(*inliers, *coefficients);

    double a = coefficients->values[0];
    double b = coefficients->values[1];
    double c = coefficients->values[2];
    double d = coefficients->values[3];

    ground_orientation_ = tf::createQuaternionMsgFromRollPitchYaw(
        atan2(b, c), -atan2(a, c), atan2(b, a));
    ground_dist_ = -(a * pose_.pose.position.x + b * pose_.pose.position.y +
                     c * pose_.pose.position.z + d) /
                   (a * a + b * b + c * c);
    closest_point_on_ground_.x = pose_.pose.position.x + a * ground_dist_;
    closest_point_on_ground_.y = pose_.pose.position.y + b * ground_dist_;
    closest_point_on_ground_.z = pose_.pose.position.z + c * ground_dist_;

    // Assume estimate to be valid if at least a certain number of inliers
    if (inliers->indices.size() > min_plane_points_ &&
        inliers->indices.size() > min_plane_percentage_ * ground_cloud_.width) {
      ground_detected_ = true;
    } else {
      ground_detected_ = false;
    }

    // Extract patch size from inliers
    double xmin = HUGE_VAL;
    double xmax = -HUGE_VAL;
    double ymin = HUGE_VAL;
    double ymax = -HUGE_VAL;
    std::vector<int> indices = inliers->indices;
    for (int i = 0; i < indices.size(); i++) {
      if (ground_cloud_.points[indices[i]].x > xmax) {
        xmax = ground_cloud_.points[indices[i]].x;
      }
      if (ground_cloud_.points[indices[i]].x < xmin) {
        xmin = ground_cloud_.points[indices[i]].x;
      }
      if (ground_cloud_.points[indices[i]].y > ymax) {
        ymax = ground_cloud_.points[indices[i]].y;
      }
      if (ground_cloud_.points[indices[i]].y < ymin) {
        ymin = ground_cloud_.points[indices[i]].y;
      }
    }

    double h_tol = 0.2;
    double xy_tol = 0.1;
    double ground_height = pose_.pose.position.z - std::abs(ground_dist_);
    bool same_surface = false;
    int heights_length = ground_heights_.size();
    if (ground_detected_) {
      if (heights_length == 0) {
        ground_heights_.push_back(ground_height);
        ground_xmax_.push_back(xmax);
        ground_xmin_.push_back(xmin);
        ground_ymax_.push_back(ymax);
        ground_ymin_.push_back(ymin);
      } else {
        int i = 0;
        for (std::vector<double>::iterator it = ground_heights_.begin();
             it != ground_heights_.end(); ++it) {
          if (!same_surface && ground_heights_[i] >= ground_height - h_tol &&
              ground_heights_[i] <= ground_height + h_tol) {
            if (((xmax <= ground_xmax_[i] + xy_tol &&
                  xmax >= ground_xmin_[i] - xy_tol) ||
                 (xmin <= ground_xmax_[i] + xy_tol &&
                  xmin >= ground_xmin_[i] - xy_tol)) &&
                ((ymax <= ground_ymax_[i] + xy_tol &&
                  ymax >= ground_ymin_[i] - xy_tol) ||
                 (ymin <= ground_ymax_[i] + xy_tol &&
                  ymin >= ground_ymin_[i] - xy_tol))) {
              if (ground_xmax_[i] < xmax) {
                ground_xmax_[i] = xmax;
              }
              if (ground_ymax_[i] < ymax) {
                ground_ymax_[i] = ymax;
              }
              if (ground_xmin_[i] > xmin) {
                ground_xmin_[i] = xmin;
              }
              if (ground_ymin_[i] > ymin) {
                ground_ymin_[i] = ymin;
              }
              ground_heights_[i] =
                  0.8 * ground_heights_[i] + 0.2 * ground_height;
              same_surface = true;
            }
          }
          i++;
        }
        if (!same_surface) {
          ground_heights_.push_back(ground_height);
          ground_xmax_.push_back(xmax);
          ground_xmin_.push_back(xmin);
          ground_ymax_.push_back(ymax);
          ground_ymin_.push_back(ymin);
        }
      }
    }

    ROS_DEBUG("Plane fit %2.2fms. Inlier %d.Ground detected: %d",
              (std::clock() - start_time) / (double) (CLOCKS_PER_SEC / 1000),
               static_cast<int>(inliers->indices.size()), ground_detected_);
  }else{
    ground_detected_ = false;
  }
}

int GroundDetector::getMinFlightElevationIndex(
    geometry_msgs::PoseStamped current_pose, double min_flight_height,
    int resolution) {
  // discard all bins which would lead too close to the ground
  int e_min_idx = -1;
  int e_max = floor(V_FOV / 2);
  int e_min;

  if (!is_near_min_height_ && over_obstacle_ &&
      current_pose.pose.position.z < min_flight_height_ + 0.2) {
    is_near_min_height_ = true;
  }
  if (is_near_min_height_ && over_obstacle_ &&
      current_pose.pose.position.z > min_flight_height_ + 0.5) {
    is_near_min_height_ = false;
  }
  if (!too_low_ && over_obstacle_ &&
      current_pose.pose.position.z <= min_flight_height_) {
    too_low_ = true;
  }
  if (too_low_ && over_obstacle_ &&
      current_pose.pose.position.z > min_flight_height_ + 0.2) {
    too_low_ = false;
  }

  if (over_obstacle_ && too_low_) {
    e_min_idx = elevationAngletoIndex(e_max, resolution);
  }
  if (over_obstacle_ && is_near_min_height_ && !too_low_) {
    e_min = 0;
    e_min_idx = elevationAngletoIndex(e_min, resolution);
  }

  return e_min_idx;
}

double GroundDetector::getMinFlightHeight(
    geometry_msgs::PoseStamped current_pose,
    geometry_msgs::TwistStamped curr_vel, bool over_obstacle_old,
    double min_flight_height_old, double margin_old) {
  int i = 0;
  over_obstacle_ = false;
  min_flight_height_ = -HUGE_VAL;
  double begin_rise_temp;

  for (std::vector<double>::iterator it = ground_heights_.begin();
       it != ground_heights_.end(); ++it) {
    double flight_height = ground_heights_[i] + min_dist_to_ground_;
    double begin_rise;
    if (over_obstacle_old && flight_height > min_flight_height_old - 0.1 &&
        flight_height < min_flight_height_old + 0.1) {
      begin_rise = margin_old;
    } else if (flight_height > current_pose.pose.position.z) {
      double dist_diff = flight_height - current_pose.pose.position.z;
      int e_max = floor(V_FOV / 2);
      e_max = e_max - e_max % ALPHA_RES;
      e_max = e_max + (ALPHA_RES - e_max % ALPHA_RES);  //[-80,+90]
      begin_rise = dist_diff / tan(e_max * M_PI / 180.0);
    } else {
      begin_rise = 0.0;
    }
    double xmin_begin_rise = 0;
    double xmax_begin_rise = begin_rise;
    double ymin_begin_rise = 0;
    double ymax_begin_rise = begin_rise;
    if (curr_vel.twist.linear.x > 0) {
      xmin_begin_rise = begin_rise;
      xmax_begin_rise = 0;
    }
    if (curr_vel.twist.linear.y > 0) {
      ymin_begin_rise = begin_rise;
      ymax_begin_rise = 0;
    }
    if (current_pose.pose.position.x < ground_xmax_[i] + xmax_begin_rise &&
        current_pose.pose.position.x > ground_xmin_[i] - xmin_begin_rise &&
        current_pose.pose.position.y < ground_ymax_[i] + ymax_begin_rise &&
        current_pose.pose.position.y > ground_ymin_[i] - ymin_begin_rise) {
      if (flight_height > min_flight_height_) {
        min_flight_height_ = flight_height;
        over_obstacle_ = true;
        begin_rise_temp = begin_rise;
      }
    }

    i++;
  }
  begin_rise_ = begin_rise_temp;

  return min_flight_height_;
}
