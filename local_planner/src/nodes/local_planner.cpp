#include "local_planner.h"

LocalPlanner::LocalPlanner() {}

LocalPlanner::~LocalPlanner() {}

// update UAV pose
void LocalPlanner::setPose(const geometry_msgs::PoseStamped msg) {
  pose_.header = msg.header;
  pose_.pose.position = msg.pose.position;
  pose_.pose.orientation = msg.pose.orientation;

  if (set_first_yaw_){
    curr_yaw_ = tf::getYaw(msg.pose.orientation);
  }

  if(take_off_){
    take_off_pose_.header = msg.header;
    take_off_pose_.pose.position = msg.pose.position;
    take_off_pose_.pose.orientation = msg.pose.orientation;
    take_off_ = false;
  }

  setVelocity();
  setLimitsBoundingBox();
}

// reset cloud and histogram counter variables when new cloud comes in
void LocalPlanner::resetHistogramCounter() {
  new_cloud_ = true;
  n_call_hist_ = 0;
}

// set parameters changed by dynamic rconfigure
void LocalPlanner::dynamicReconfigureSetParams(avoidance::LocalPlannerNodeConfig & config,uint32_t level){
  min_box_x_ = config.min_box_x_;
  max_box_x_ = config.max_box_x_;
  min_box_y_ = config.min_box_y_;
  max_box_y_ = config.max_box_y_;
  min_box_z_ = config.min_box_z_;
  max_box_z_ = config.max_box_z_;
  min_dist_to_ground_ = config.min_dist_to_ground_;
  goal_cost_param_ = config.goal_cost_param_;
  smooth_cost_param_ = config.smooth_cost_param_;
  min_speed_ = config.min_speed_;
  max_speed_ = config.max_speed_;
  max_accel_z_ = config.max_accel_z_;
  stop_in_front_ = config.stop_in_front_;
  keep_distance_ = config.keep_distance_;
  ground_inlier_angle_threshold_ = config.ground_inlier_angle_threshold_;
  ground_inlier_distance_threshold_ = config.ground_inlier_distance_threshold_;
  no_progress_slope_ = config.no_progress_slope_ ;
  progress_slope_  = config.progress_slope_ ;
  rise_factor_no_progress_ = config.rise_factor_no_progress_;

  if (goal_z_param!= config.goal_z_param) {
    goal_z_param = config.goal_z_param;
    setGoal();
  }
  use_ground_detection_ = config.use_ground_detection_;
  box_size_increase_ = config.box_size_increase_;
}

// log Data
void LocalPlanner::logData() {
  if (!reach_altitude_) {
    time_t t = time(0);
    struct tm * now = localtime(&t);
    std::string buffer(80, '\0');
    strftime(&buffer[0], buffer.size(), "%F-%H-%M", now);
    log_name_ = buffer;

  } else {
    std::ofstream myfile(("LocalPlanner_" + log_name_).c_str(), std::ofstream::app);
    myfile << pose_.header.stamp.sec << "\t" << pose_.header.stamp.nsec << "\t" << pose_.pose.position.x << "\t" << pose_.pose.position.y << "\t" << pose_.pose.position.z << "\t" << local_planner_mode_ << "\t" << reached_goal_ << "\t"
        << box_size_increase_ << "\t" << use_ground_detection_ << "\t" << obstacle_ << "\t" << no_progress_rise_ << "\t" << over_obstacle_ << "\t" << too_low_ << "\t" << is_near_min_height_ << "\n";
    myfile.close();

    if (print_height_map_) {
      std::ofstream myfile(("InternalHeightMap_" + log_name_).c_str(), std::ofstream::app);
      myfile << pose_.header.stamp.sec << "\t" << pose_.header.stamp.nsec <<"\t" << 0 <<"\t" << 0 <<"\t" << 0 << "\n";
      int i = 0;
      for (std::vector<double>::iterator it = ground_heights_.begin(); it != ground_heights_.end(); ++it) {
        myfile << ground_heights_[i] << "\t" << ground_xmin_[i] << "\t" << ground_xmax_[i] << "\t" << ground_ymin_[i] << "\t" << ground_ymax_[i] << "\n";
        i++;
      }
      myfile.close();
    }
  }
}

// update UAV velocity
void LocalPlanner::setVelocity() {
  velocity_x_ = curr_vel_.twist.linear.x;
  velocity_y_ = curr_vel_.twist.linear.y;
  velocity_z_ = curr_vel_.twist.linear.z;
  velocity_mod_ = sqrt(pow(velocity_x_,2) + pow(velocity_y_,2) + pow(velocity_z_,2));
}

// update bounding box limit coordinates around a new UAV pose
void LocalPlanner::setLimitsBoundingBox() {
  min_box_.x = pose_.pose.position.x - min_box_x_;
  min_box_.y = pose_.pose.position.y - min_box_y_;
  min_box_.z = pose_.pose.position.z - min_box_z_;
  max_box_.x = pose_.pose.position.x + max_box_x_;
  max_box_.y = pose_.pose.position.y + max_box_y_;
  max_box_.z = pose_.pose.position.z + max_box_z_;

  min_groundbox_.x = pose_.pose.position.x - min_groundbox_x_;
  min_groundbox_.y = pose_.pose.position.y - min_groundbox_y_;
  min_groundbox_.z = pose_.pose.position.z - min_dist_to_ground_ - min_groundbox_z_;
  max_groundbox_.x = pose_.pose.position.x + max_groundbox_x_;
  max_groundbox_.y = pose_.pose.position.y + max_groundbox_y_;
  max_groundbox_.z = pose_.pose.position.z ;
}

// set mission goal
void LocalPlanner::setGoal() {
  goal_.x = goal_x_param;
  goal_.y = goal_y_param;
  goal_.z = goal_z_param;
  reached_goal_ = false;
  ROS_INFO("===== Set Goal ======: [%f, %f, %f].", goal_.x, goal_.y, goal_.z);
  initGridCells(&path_waypoints_);
}

bool LocalPlanner::isPointWithinHistogramBox(pcl::PointCloud<pcl::PointXYZ>::iterator pcl_it) {
  return (pcl_it->x) < max_box_.x && (pcl_it->x) > min_box_.x && (pcl_it->y) < max_box_.y && (pcl_it->y) > min_box_.y && (pcl_it->z) < max_box_.z && (pcl_it->z) > min_box_.z;
}

bool LocalPlanner::isPointWithinGroundBox(pcl::PointCloud<pcl::PointXYZ>::iterator pcl_it) {
  return (pcl_it->x) < max_groundbox_.x && (pcl_it->x) > min_groundbox_.x && (pcl_it->y) < max_groundbox_.y && (pcl_it->y) > min_groundbox_.y && (pcl_it->z) < max_groundbox_.z && (pcl_it->z) > min_groundbox_.z;
}

// fit plane through groud cloud
void LocalPlanner::fitPlane() {
  std::clock_t start_time = std::clock();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  cloud->width = ground_cloud_.width;
  cloud->height = ground_cloud_.height;
  cloud->points = ground_cloud_.points;

  if (ground_cloud_.width > 100) {
    ROS_INFO("Cloud size %d. Plane fit in progress...", ground_cloud_.width);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation < pcl::PointXYZ > seg;
    seg.setOptimizeCoefficients(true);


    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(ground_inlier_distance_threshold_);
    seg.setInputCloud(cloud);
    seg.setAxis (Eigen::Vector3f (0.0, 0.0, 1.0));
    seg.setEpsAngle (ground_inlier_angle_threshold_*PI/180.0);
    seg.segment(*inliers, *coefficients);

    double a = coefficients->values[0];
    double b = coefficients->values[1];
    double c = coefficients->values[2];
    double d = coefficients->values[3];

    ground_orientation_ = tf::createQuaternionMsgFromRollPitchYaw (atan2(b,c), -atan2(a,c), atan2(b,a));
    ground_dist_ = -(a*pose_.pose.position.x + b*pose_.pose.position.y + c*pose_.pose.position.z + d)/(a*a + b*b + c*c);
    closest_point_on_ground_.x = pose_.pose.position.x + a*ground_dist_;
    closest_point_on_ground_.y = pose_.pose.position.y + b*ground_dist_;
    closest_point_on_ground_.z = pose_.pose.position.z + c*ground_dist_;

   //Assume estimate to be valid if 60% of the points are inliers
    if(inliers->indices.size()>0.6*ground_cloud_.width){
      ground_detected_ = true;
    }else{
      ground_detected_ = false;
    }

    // Extract patch size from inliers
    double xmin = 10000;
    double xmax = -10000;
    double ymin = 10000;
    double ymax = -10000;
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
        for (std::vector<double>::iterator it = ground_heights_.begin(); it != ground_heights_.end(); ++it) {
          if (!same_surface && ground_heights_[i] >= ground_height - h_tol && ground_heights_[i] <= ground_height + h_tol) {
            if (((xmax <= ground_xmax_[i] + xy_tol && xmax >= ground_xmin_[i] - xy_tol) || (xmin <= ground_xmax_[i] + xy_tol && xmin >= ground_xmin_[i] - xy_tol))
                && ((ymax <= ground_ymax_[i] + xy_tol && ymax >= ground_ymin_[i] - xy_tol) || (ymin <= ground_ymax_[i] + xy_tol && ymin >= ground_ymin_[i] - xy_tol))) {
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
              ground_heights_[i] = 0.8 * ground_heights_[i] + 0.2 * ground_height;
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

    ROS_INFO("Plane fit %2.2fms. Inlier %d.", (std::clock() - start_time) / (double) (CLOCKS_PER_SEC / 1000), inliers->indices.size());
  }else{
    ground_detected_ = false;
  }
}

// trim the point cloud so that only points inside the bounding box are considered and
void LocalPlanner::filterPointCloud(pcl::PointCloud<pcl::PointXYZ>& complete_cloud) {
  calculateFOV();
  std::clock_t start_time = std::clock();
  pcl::PointCloud<pcl::PointXYZ>::iterator pcl_it;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp1(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp2(new pcl::PointCloud<pcl::PointXYZ>);
  final_cloud_.points.clear();
  ground_cloud_.points.clear();
  distance_to_closest_point_ = 1000.0f;
  double min_realsense_dist = 0.2;
  float distance;

  if (new_cloud_){
    complete_cloud_ = complete_cloud;
  }

  for (pcl_it = complete_cloud_.begin(); pcl_it != complete_cloud_.end(); ++pcl_it) {
    // Check if the point is invalid
    if (!std::isnan(pcl_it->x) && !std::isnan(pcl_it->y) && !std::isnan(pcl_it->z)) {
      if (isPointWithinHistogramBox(pcl_it)) {
        distance = computeL2Dist(pose_, pcl_it);
        if (distance > min_realsense_dist) {
          cloud_temp1->points.push_back(pcl::PointXYZ(pcl_it->x, pcl_it->y, pcl_it->z));
          if (distance < distance_to_closest_point_) {
            distance_to_closest_point_ = distance;
            closest_point_.x = pcl_it->x;
            closest_point_.y = pcl_it->y;
            closest_point_.z = pcl_it->z;
          }
        }
      }
      if (use_ground_detection_) {
        if (isPointWithinGroundBox(pcl_it)) {
          cloud_temp2->points.push_back(pcl::PointXYZ(pcl_it->x, pcl_it->y, pcl_it->z));
        }
      }
    }
  }

  //increase safety radius if too close to the wall
  if(distance_to_closest_point_ < 1.7 && cloud_temp1->points.size() > 160){
    safety_radius_ = 25+ 2*alpha_res;
    std::cout<<"Increased safety radius!\n";
  }
  if(distance_to_closest_point_ > 2.5 && distance_to_closest_point_ < 1000 && cloud_temp1->points.size() > 160){
    safety_radius_ = 25;
    std::cout<<"Lowered safety radius!\n";
  }

  final_cloud_.header.stamp = complete_cloud.header.stamp;
  final_cloud_.header.frame_id = complete_cloud.header.frame_id;
  final_cloud_.width = cloud_temp1->points.size();
  final_cloud_.height = 1;
  final_cloud_.points = cloud_temp1->points;

  ground_cloud_.header.stamp = complete_cloud.header.stamp;
  ground_cloud_.header.frame_id = complete_cloud.header.frame_id;
  ground_cloud_.width = cloud_temp2->points.size();
  ground_cloud_.height = 1;
  ground_cloud_.points = cloud_temp2->points;

  ROS_INFO("Point cloud cropped in %2.2fms. Cloud size %d.", (std::clock() - start_time) / (double)(CLOCKS_PER_SEC / 1000), final_cloud_.width);
  cloud_time_.push_back((std::clock() - start_time) / (double)(CLOCKS_PER_SEC / 1000));

  if(!reach_altitude_){
    std::cout << "\033[1;32m Reach height ("<<take_off_pose_.pose.position.z + goal_.z - 0.5<<") first: Go fast\n \033[0m";
    local_planner_mode_ = 0;
    goFast();
  }else if (cloud_temp1->points.size() > 160 && stop_in_front_ && reach_altitude_) {
    obstacle_ = true;
    std::cout << "\033[1;32m There is an Obstacle Ahead stop in front\n \033[0m";
    local_planner_mode_ = 3;
    stopInFrontObstacles();
  } else {
    if ((distance_to_closest_point_ < 1.5 || back_off_) && reach_altitude_ && cloud_temp1->points.size() > 160) {
      local_planner_mode_ = 4;
      std::cout << "\033[1;32m There is an Obstacle too close! Back off\n \033[0m";
      if(!back_off_){
        back_off_point_ = closest_point_;
        back_off_ = true;
      }
      backOff();

    } else {
      createPolarHistogram();  //TODO: Maybe also discard points is cloud is too small
      first_brake_ = true;
    }
  }

  //fit horizontal plane for ground estimation
  if (use_ground_detection_ && reach_altitude_ && new_cloud_){
    fitPlane();
  }
  new_cloud_ = false;
}

float distance3DCartesian(geometry_msgs::Point a, geometry_msgs::Point b) {
  return sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) + (a.z-b.z)*(a.z-b.z));
}

float distance2DPolar(int e1, int z1, int e2, int z2){
  return sqrt(pow((e1-e2),2) + pow((z1-z2),2));
}

float computeL2Dist(geometry_msgs::PoseStamped pose, pcl::PointCloud<pcl::PointXYZ>::iterator pcl_it) {
  return sqrt(pow(pose.pose.position.x - pcl_it->x, 2) + pow(pose.pose.position.y - pcl_it->y, 2) + pow(pose.pose.position.z - pcl_it->z, 2));
}


bool LocalPlanner::obstacleAhead() {
  if(obstacle_){
    return true;
  } else{
    return false;
  }
}

bool LocalPlanner::groundDetected() {
  if(ground_detected_){
    return true;
  } else{
    return false;
  }
}

// Calculate FOV. Azimuth angle is wrapped, elevation is not!
void LocalPlanner::calculateFOV() {
  tf::Quaternion q(pose_.pose.orientation.x, pose_.pose.orientation.y, pose_.pose.orientation.z, pose_.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  double z_FOV_max = std::round((-yaw * 180.0 / PI + h_fov / 2.0 + 270.0) / alpha_res) - 1;
  double z_FOV_min = std::round((-yaw * 180.0 / PI - h_fov / 2.0 + 270.0) / alpha_res) - 1;
  e_FOV_max_ = std::round((-pitch * 180.0 / PI + v_fov / 2.0 + 90.0) / alpha_res) - 1;
  e_FOV_min_ = std::round((-pitch * 180.0 / PI - v_fov / 2.0 + 90.0) / alpha_res) - 1;

  if (z_FOV_max >= grid_length_z && z_FOV_min >= grid_length_z) {
    z_FOV_max -= grid_length_z;
    z_FOV_min -= grid_length_z;
  }
  if (z_FOV_max < 0 && z_FOV_min < 0) {
    z_FOV_max += grid_length_z;
    z_FOV_min += grid_length_z;
  }

  z_FOV_idx_.clear();
  if (z_FOV_max >= grid_length_z && z_FOV_min < grid_length_z) {
    for (int i = 0; i < z_FOV_max - grid_length_z; i++) {
      z_FOV_idx_.push_back(i);
    }
    for (int i = z_FOV_min; i < grid_length_z; i++) {
      z_FOV_idx_.push_back(i);
    }
  } else if (z_FOV_min < 0 && z_FOV_max >= 0) {
    for (int i = 0; i < z_FOV_max; i++) {
      z_FOV_idx_.push_back(i);
    }
    for (int i = z_FOV_min + grid_length_z; i < grid_length_z; i++) {
      z_FOV_idx_.push_back(i);
    }
  } else {
    for (int i = z_FOV_min; i < z_FOV_max; i++) {
      z_FOV_idx_.push_back(i);
    }
  }
}

// fill the 2D polar histogram with the points from the filtered point cloud
void LocalPlanner::createPolarHistogram() {
  std::clock_t start_time = std::clock();
  float dist;
  float age;
  double n_points = 0;
  int n_split = 4;
  geometry_msgs::Point temp;
  geometry_msgs::Point temp_array[n_split];

  //Build Estimate of Histogram from old Histogram and Movement
  polar_histogram_est_ = Histogram(2 * alpha_res);
  reprojected_points_.points.clear();
  reprojected_points_.header.stamp = final_cloud_.header.stamp;
  reprojected_points_.header.frame_id = "world";

  for (int e = 0; e < grid_length_e; e++) {
    for (int z = 0; z < grid_length_z; z++) {
      if (polar_histogram_old_.get_bin(e, z) != 0) {
        n_points++;
        //transform from array index to angle
        double beta_e = (e + 1.0) * alpha_res - 90 - alpha_res/2.0;
        double beta_z = (z + 1.0) * alpha_res - 180 - alpha_res/2.0;
        //transform from Polar to Cartesian
        temp_array[0] = fromPolarToCartesian(beta_e + alpha_res / 2, beta_z + alpha_res / 2, polar_histogram_old_.get_dist(e, z), position_old_);
        temp_array[1] = fromPolarToCartesian(beta_e - alpha_res / 2, beta_z + alpha_res / 2, polar_histogram_old_.get_dist(e, z), position_old_);
        temp_array[2] = fromPolarToCartesian(beta_e + alpha_res / 2, beta_z - alpha_res / 2, polar_histogram_old_.get_dist(e, z), position_old_);
        temp_array[3] = fromPolarToCartesian(beta_e - alpha_res / 2, beta_z - alpha_res / 2, polar_histogram_old_.get_dist(e, z), position_old_);

        for (int i = 0; i < n_split; i++) {
          dist = distance3DCartesian(pose_.pose.position, temp_array[i]);
          age = polar_histogram_old_.get_age(e, z);

          if (dist < 2*max_box_x_ && dist > 0.3 && age < age_lim) {
            reprojected_points_.points.push_back(pcl::PointXYZ(temp_array[i].x, temp_array[i].y, temp_array[i].z));
            int beta_z_new = floor(atan2(temp_array[i].x - pose_.pose.position.x, temp_array[i].y - pose_.pose.position.y) * 180.0 / PI);  //(-180. +180]
            int beta_e_new = floor(
                atan((temp_array[i].z - pose_.pose.position.z) / sqrt((temp_array[i].x - pose_.pose.position.x) * (temp_array[i].x - pose_.pose.position.x) + (temp_array[i].y - pose_.pose.position.y) * (temp_array[i].y - pose_.pose.position.y)))
                    * 180.0 / PI);  //(-90.+90)

            beta_e_new += 90;
            beta_z_new += 180;

            beta_z_new = beta_z_new + ((2 * alpha_res) - (beta_z_new % (2 * alpha_res)));  //[-170,+180]
            beta_e_new = beta_e_new + ((2 * alpha_res) - (beta_e_new % (2 * alpha_res)));  //[-80,+90]

            int e_new = (beta_e_new) / (2 * alpha_res) - 1;  //[0,17]
            int z_new = (beta_z_new) / (2 * alpha_res) - 1;  //[0,35]

            polar_histogram_est_.set_bin(e_new, z_new, polar_histogram_est_.get_bin(e_new, z_new) + 1.0 / n_split);
            polar_histogram_est_.set_age(e_new, z_new, polar_histogram_est_.get_age(e_new, z_new) + 1.0 / n_split * age);
            polar_histogram_est_.set_dist(e_new, z_new, polar_histogram_est_.get_dist(e_new, z_new) + 1.0 / n_split * dist);
          }
        }
      }
    }
  }

  for (int e = 0; e < grid_length_e / 2; e++) {
    for (int z = 0; z < grid_length_z / 2; z++) {
      if (polar_histogram_est_.get_bin(e, z) >= min_bin) {
        polar_histogram_est_.set_dist(e, z, polar_histogram_est_.get_dist(e, z) / polar_histogram_est_.get_bin(e, z));
        polar_histogram_est_.set_age(e, z, polar_histogram_est_.get_age(e, z) / polar_histogram_est_.get_bin(e, z));
        polar_histogram_est_.set_bin(e, z, 1);
      } else {
        polar_histogram_est_.set_dist(e, z, 0);
        polar_histogram_est_.set_age(e, z, 0);
        polar_histogram_est_.set_bin(e, z, 0);
      }
    }
  }

  //Upsample histogram estimate
  polar_histogram_est_.upsample();

  //Generate new histogram from pointcloud
  polar_histogram_.setZero();
  pcl::PointCloud<pcl::PointXYZ>::const_iterator it;

  for( it = final_cloud_.begin(); it != final_cloud_.end(); ++it) {
    temp.x = it->x;
    temp.y = it->y;
    temp.z = it->z;
    dist = distance3DCartesian(pose_.pose.position,temp);

    int beta_z = floor((atan2(temp.x - pose_.pose.position.x, temp.y - pose_.pose.position.y) * 180.0 / PI));  //(-180. +180]
    int beta_e = floor((atan((temp.z - pose_.pose.position.z) / sqrt((temp.x - pose_.pose.position.x) * (temp.x - pose_.pose.position.x) + (temp.y - pose_.pose.position.y) * (temp.y - pose_.pose.position.y))) * 180.0 / PI));  //(-90.+90)

    beta_e += 90;  //[0,360]
    beta_z += 180; //[0,180]

    beta_z = beta_z + (alpha_res - beta_z % alpha_res);  //[10,360]
    beta_e = beta_e + (alpha_res - beta_e % alpha_res);  //[10,180]

    int e = beta_e / alpha_res - 1;  //[0,17]
    int z = beta_z/ alpha_res - 1;  //[0,35]

    polar_histogram_.set_bin(e, z, polar_histogram_.get_bin(e, z) + 1);
    polar_histogram_.set_dist(e, z, polar_histogram_.get_dist(e, z) + dist);
  }

  //Normalize and get mean in distance bins
  for (int e = 0; e < grid_length_e; e++) {
    for (int z = 0; z < grid_length_z; z++) {
      if (polar_histogram_.get_bin(e, z) > 0) {
        polar_histogram_.set_dist(e, z, polar_histogram_.get_dist(e, z) / polar_histogram_.get_bin(e, z));
        polar_histogram_.set_bin(e, z, 1);
      }
    }
  }

  //Combine to New binary histogram
  bool hist_is_empty = true;
  for (int e = 0; e < grid_length_e; e++) {
    for (int z = 0; z < grid_length_z; z++) {
      if (std::find(z_FOV_idx_ .begin(), z_FOV_idx_ .end(), z) != z_FOV_idx_ .end() && e > e_FOV_min_ && e < e_FOV_max_) {  //inside FOV
        if (polar_histogram_.get_bin(e, z) > 0) {
          polar_histogram_.set_age(e, z, 1);
          hist_is_empty = false;
        }
      } else {
        if (polar_histogram_est_.get_bin(e, z) > 0) {
          if (waypoint_outside_FOV_) {
            polar_histogram_.set_age(e, z, polar_histogram_est_.get_age(e, z));
          } else {
            polar_histogram_.set_age(e, z, polar_histogram_est_.get_age(e, z) + 1);
          }
          hist_is_empty = false;
        }
        if (polar_histogram_.get_bin(e, z) > 0) {
          polar_histogram_.set_age(e, z, 1);
          hist_is_empty = false;
        }
        if (polar_histogram_est_.get_bin(e, z) > 0 && polar_histogram_.get_bin(e, z) == 0) {
          polar_histogram_.set_bin(e, z, polar_histogram_est_.get_bin(e, z));
          polar_histogram_.set_dist(e, z, polar_histogram_est_.get_dist(e, z));
        }
      }
    }
  }

  //Update old histogram
  polar_histogram_old_.setZero();
  polar_histogram_old_ = polar_histogram_;
  n_call_hist_ +=1;

  //Statistics
  ROS_INFO("Polar histogram created in %2.2fms.", (std::clock() - start_time) / (double) (CLOCKS_PER_SEC / 1000));
  polar_time_.push_back((std::clock() - start_time) / (double) (CLOCKS_PER_SEC / 1000));

  //decide how to proceed
  if(hist_is_empty && n_call_hist_==1 && box_size_increase_){
    min_box_.x = pose_.pose.position.x - 2 * min_box_x_;
    min_box_.y = pose_.pose.position.y - 2 * min_box_y_;
    min_box_.z = pose_.pose.position.z - 2 * min_box_z_;
    max_box_.x = pose_.pose.position.x + 2 * max_box_x_;
    max_box_.y = pose_.pose.position.y + max_box_y_;
    max_box_.z = pose_.pose.position.z + max_box_z_;
    std::cout << "\033[1;33m Box size increased!\n \033[0m";
    filterPointCloud(complete_cloud_);

  }else if((hist_is_empty && n_call_hist_>1 &&box_size_increase_) || (hist_is_empty && !box_size_increase_)){
    obstacle_ = false;
    std::cout << "\033[1;32m There is NO Obstacle Ahead go Fast\n \033[0m";
    local_planner_mode_ = 1;
    goFast();
  }

  if(!hist_is_empty && reach_altitude_){
    obstacle_ = true;
    std::cout << "\033[1;32m There is an Obstacle Ahead use Histogram\n \033[0m";
    local_planner_mode_ = 2;
    findFreeDirections();
  }
}

void LocalPlanner::printHistogram(Histogram hist){
  for (int e_ind = 0; e_ind < grid_length_e; e_ind++) {
    for (int z_ind = 0; z_ind < grid_length_z; z_ind++) {
      if (std::find(z_FOV_idx_ .begin(), z_FOV_idx_ .end(), z_ind) != z_FOV_idx_ .end() && e_ind > e_FOV_min_ && e_ind < e_FOV_max_) {
        std::cout << "\033[1;32m" << hist.get_bin(e_ind, z_ind) << " \033[0m";
      } else {
        std::cout << hist.get_bin(e_ind, z_ind) << " ";
      }
    }
    std::cout << "\n";
  }
  std::cout << "--------------------------------------\n";
}

// initialize GridCell message
void LocalPlanner::initGridCells(nav_msgs::GridCells *cell) {
  cell->cells.clear();
  cell->header.stamp = ros::Time::now();
  cell->header.frame_id = "/world";
  cell->cell_width = alpha_res;
  cell->cell_height = alpha_res;
  cell->cells = {};
}

// search for free directions in the 2D polar histogram with a moving window approach
void LocalPlanner::findFreeDirections() {
  std::clock_t start_time = std::clock();
  int n = floor(safety_radius_ / alpha_res);  //safety radius
  int a = 0, b = 0;
  bool free = true;
  bool corner = false;
  bool height_reject = false;
  geometry_msgs::Point p;
  cost_path_candidates_.clear();

  initGridCells(&path_candidates_);
  initGridCells(&path_rejected_);
  initGridCells(&path_blocked_);
  initGridCells(&path_selected_);
  initGridCells(&path_ground_);

  updateCostParameters();

  // discard all bins which would lead too close to the ground
  int e_min_idx = -1;
  if (use_ground_detection_) {
    getMinFlightHeight();
    int e_max = floor(v_fov / 2);
    int e_min;

    if(!is_near_min_height_ && over_obstacle_ && pose_.pose.position.z < min_flight_height_ + 0.2){
      is_near_min_height_ = true;
    }
    if(is_near_min_height_ && over_obstacle_ && pose_.pose.position.z > min_flight_height_ + 0.5){
      is_near_min_height_ = false;
    }
    if(!too_low_ && over_obstacle_ && pose_.pose.position.z <= min_flight_height_){
       too_low_ = true;
     }
     if(too_low_  && over_obstacle_ && pose_.pose.position.z > min_flight_height_ + 0.2){
       too_low_ = false;
     }

    if (over_obstacle_ && too_low_) {
      e_max = e_max+90;
      e_max = e_max - e_max % alpha_res;
      e_max = e_max + (alpha_res - e_max % alpha_res);  //[10,180]
      e_min_idx = (e_max) / alpha_res - 1;  //[0,17]
      std::cout << "\033[1;36m Too low, discard points under elevation " << e_max<< "\n \033[0m";
    }
    if (over_obstacle_ && is_near_min_height_ && !too_low_) {
      e_min = 0;
      //e_min = e_min + (alpha_res - e_min % alpha_res);  //[-80,+90]
      e_min_idx = (90 + e_min) / alpha_res - 1;  //[0,17]
      std::cout << "\033[1;36m Prevent down flight, discard points under elevation " << e_min<< "\n \033[0m";
    }
  }

  //determine which bins are candidates
  for (int e = 0; e < grid_length_e; e++) {
    for (int z = 0; z < grid_length_z; z++) {
      for (int i = (e - n); i <= (e + n); i++) {
        for (int j = (z - n); j <= (z + n); j++) {

          free = true;
          corner = false;
          height_reject = false;

          // Elevation index < 0
          if(i < 0 && j >= 0 && j < grid_length_z) {
            a = -i;
            b = grid_length_z - j - 1;
          }
          // Azimuth index < 0
          else if(j < 0 && i >= 0 && i < grid_length_e) {
            b = j+grid_length_z;
          }
          // Elevation index > grid_length_e
          else if(i >= grid_length_e && j >= 0 && j < grid_length_z) {
            a = grid_length_e - (i % (grid_length_e - 1));
            b = grid_length_z - j - 1;
          }
          // Azimuth index > grid_length_z
          else if(j >= grid_length_z && i >= 0 && i < grid_length_e) {
            b = j - grid_length_z;
          }
          // Elevation and Azimuth index both within histogram
          else if( i >= 0 && i < grid_length_e && j >= 0 && j < grid_length_z) {
            a = i;
            b = j;
          }
          // Elevation and azimuth index both < 0 OR elevation index > grid_length_e and azimuth index <> grid_length_z
          // OR elevation index < 0 and azimuth index > grid_length_z OR elevation index > grid_length_e and azimuth index < 0.
          // These cells are not part of the polar histogram.
          else {
            corner = true;
          }

          if(!corner) {
            if(polar_histogram_.get_bin(a,b) != 0) {
              free = false;
              break;
            }
          }
        }

        if(!free)
          break;
      }

      //reject points which lead the drone too close to the ground
      if (use_ground_detection_ && over_obstacle_) {
        if (e <= e_min_idx) {
          height_reject = true;
        }
      }

      if (free && !height_reject) {
        p.x = e * alpha_res + alpha_res - 90;
        p.y = z * alpha_res + alpha_res - 180;
        p.z = 0;
        path_candidates_.cells.push_back(p);
        cost_path_candidates_.push_back(costFunction((int) p.x, (int) p.y));
      } else if (!free && polar_histogram_.get_bin(e, z) != 0 && !height_reject) {
        p.x = e * alpha_res + alpha_res - 90;
        p.y = z * alpha_res + alpha_res - 180;
        p.z = 0;
        path_rejected_.cells.push_back(p);
      } else if (height_reject) {
        p.x = e * alpha_res + alpha_res - 90;
        p.y = z * alpha_res + alpha_res - 180;
        p.z = 0;
        path_ground_.cells.push_back(p);
      } else {
        p.x = e * alpha_res + alpha_res - 90;
        p.y = z * alpha_res + alpha_res - 180;
        p.z = 0;
        path_blocked_.cells.push_back(p);
      }
    }
  }

  ROS_INFO("Path candidates calculated in %2.2fms.",(std::clock() - start_time) / (double)(CLOCKS_PER_SEC / 1000));
  free_time_.push_back((std::clock() - start_time) / (double)(CLOCKS_PER_SEC / 1000));

  calculateCostMap();
}

//calculate the correct weight between fly over and fly around
void LocalPlanner::updateCostParameters() {

  if (reach_altitude_) {
    double goal_dist = distance3DCartesian(pose_.pose.position, goal_);
    double goal_dist_old = distance3DCartesian(position_old_, goal_);
    double time = std::clock() / (double) (CLOCKS_PER_SEC / 1000);
    double incline = (goal_dist - goal_dist_old) / (time - integral_time_old_);
    integral_time_old_ = time;

    goal_dist_incline_.push_back(incline);
    if (goal_dist_incline_.size() > dist_incline_window_size_) {
      goal_dist_incline_.pop_front();
    }

    double sum_incline = 0;
    int n_incline = 0;
    for (int i = 0; i < goal_dist_incline_.size(); i++) {
      sum_incline += goal_dist_incline_[i];
      n_incline++;
    }
    double avg_incline = sum_incline / n_incline;
    ;
    if (avg_incline > no_progress_slope_ && goal_dist_incline_.size() == dist_incline_window_size_) {
//    height_change_cost_param_ = 0.5;
      no_progress_rise_ = true;
      smooth_cost_param_adapted_ =  smooth_cost_param_ + 0.8;
    }
    if (avg_incline < progress_slope_) {
//    height_change_cost_param_ = 5;
      no_progress_rise_ = false;
      smooth_cost_param_adapted_ =  smooth_cost_param_;
    }
  }
}

// transform polar coordinates into Cartesian coordinates
geometry_msgs::Point LocalPlanner::fromPolarToCartesian(int e, int z, double radius, geometry_msgs::Point pos) {
  geometry_msgs::Point p;
  p.x = pos.x + radius * cos(e * (PI / 180)) * sin(z * (PI / 180));  //round
  p.y = pos.y + radius * cos(e * (PI / 180)) * cos(z * (PI / 180));
  p.z = pos.z + radius * sin(e * (PI / 180));

  return p;
}

// transform a 2D polar histogram direction in a 3D Catesian coordinate point
geometry_msgs::Vector3Stamped LocalPlanner::getWaypointFromAngle(int e, int z) {
  geometry_msgs::Point p = fromPolarToCartesian(e, z, 1.0, pose_.pose.position);

  geometry_msgs::Vector3Stamped waypoint;
  waypoint.header.stamp = ros::Time::now();
  waypoint.header.frame_id = "/world";
  waypoint.vector.x = p.x;
  waypoint.vector.y = p.y;
  waypoint.vector.z = p.z;

  return waypoint;
}

// cost function according to all free directions found in the 2D polar histogram are ranked
double LocalPlanner::costFunction(int e, int z) {
  double cost;
  int waypoint_index = path_waypoints_.cells.size();

  double dist = distance3DCartesian(pose_.pose.position, goal_);
  double dist_old = distance3DCartesian(position_old_, goal_);
  geometry_msgs::Point candidate_goal = fromPolarToCartesian(e, z, dist,pose_.pose.position);
  geometry_msgs::Point old_candidate_goal = fromPolarToCartesian(path_waypoints_.cells[waypoint_index - 1].x, path_waypoints_.cells[waypoint_index - 1].y, dist_old, position_old_);
  double yaw_cost = goal_cost_param_ * sqrt((goal_.x - candidate_goal.x) * (goal_.x - candidate_goal.x) + (goal_.y - candidate_goal.y) * (goal_.y - candidate_goal.y));
  double pitch_cost = goal_cost_param_ * sqrt((goal_.z - candidate_goal.z)*(goal_.z - candidate_goal.z));
  double yaw_cost_smooth = smooth_cost_param_adapted_ * sqrt((old_candidate_goal.x - candidate_goal.x) * (old_candidate_goal .x - candidate_goal.x) + (old_candidate_goal .y - candidate_goal.y) * (old_candidate_goal .y - candidate_goal.y));
  double pitch_cost_smooth = smooth_cost_param_adapted_ * sqrt((old_candidate_goal .z - candidate_goal.z)*(old_candidate_goal .z - candidate_goal.z));

  //discurage going down
  if (candidate_goal.z<goal_.z){
    pitch_cost = 500*pitch_cost;
  }

  if(!only_yawed_){
    cost = yaw_cost + height_change_cost_param_*pitch_cost + yaw_cost_smooth + pitch_cost_smooth;
  }else{
    cost = yaw_cost + height_change_cost_param_*pitch_cost + 0.1*yaw_cost_smooth + 0.1*pitch_cost_smooth;
  }

  return cost;
}


// calculate the free direction which has the smallest cost for the UAV to travel to
void LocalPlanner::calculateCostMap() {
  std::clock_t start_time = std::clock();
  cv::sortIdx(cost_path_candidates_, cost_idx_sorted_, CV_SORT_EVERY_ROW + CV_SORT_ASCENDING);

  geometry_msgs::Point p;
  p.x = path_candidates_.cells[cost_idx_sorted_[0]].x;
  p.y = path_candidates_.cells[cost_idx_sorted_[0]].y;
  p.z = path_candidates_.cells[cost_idx_sorted_[0]].z;
  path_selected_.cells.push_back(p);
  path_waypoints_.cells.push_back(p);

  ROS_INFO("Selected path (e, z) = (%d, %d) costs %.2f. Calculated in %2.2f ms.", (int)p.x, (int)p.y, cost_path_candidates_[cost_idx_sorted_[0]], (std::clock() - start_time) / (double)(CLOCKS_PER_SEC / 1000));
  cost_time_.push_back((std::clock() - start_time) / (double)(CLOCKS_PER_SEC / 1000));

  getNextWaypoint();
}

// check that the selected direction is really free and transform it into a waypoint. Otherwise break to avoid collision with an obstacle
void LocalPlanner::getNextWaypoint() {
  int waypoint_index = path_waypoints_.cells.size();
  int e = path_waypoints_.cells[waypoint_index - 1].x;
  int z = path_waypoints_.cells[waypoint_index - 1].y;
  int e_index = (e-alpha_res+90)/alpha_res;
  int z_index = (z-alpha_res+180)/alpha_res;
  geometry_msgs::Vector3Stamped setpoint = getWaypointFromAngle(e,z);

  if(std::find(z_FOV_idx_ .begin(), z_FOV_idx_ .end(), z_index) != z_FOV_idx_ .end()){
    waypoint_outside_FOV_ = false;
  }else{
    waypoint_outside_FOV_ = true;
  }

  if (withinGoalRadius()) {
    if (over_obstacle_ && (is_near_min_height_ || too_low_)) {
      ROS_INFO("Above Goal cannot go lower: Hoovering");
      waypt_.vector.x = goal_.x;
      waypt_.vector.y = goal_.y;
      if (pose_.pose.position.z > goal_.z) {
        waypt_.vector.z = pose_.pose.position.z;
      } else {
        waypt_.vector.z = goal_.z;
      }
    } else {
      ROS_INFO("Goal Reached: Hoovering");
      waypt_.vector.x = goal_.x;
      waypt_.vector.y = goal_.y;
      waypt_.vector.z = goal_.z;
    }
  }
  else{
 	  if(checkForCollision() && pose_.pose.position.z>0.5) {
      waypt_.vector.x = 0.0;
      waypt_.vector.y = 0.0;
      waypt_.vector.z = setpoint.vector.z;

   	  ROS_INFO("Braking!!! Obstacle closer than 0.5m.");
      path_selected_.cells[path_selected_.cells.size()-1].x = 0;
      path_selected_.cells[path_selected_.cells.size()-1].y = 0;

      path_waypoints_.cells[path_waypoints_.cells.size()-1].x = 90;
      path_waypoints_.cells[path_waypoints_.cells.size()-1].y = 90;
 	  }
    else{
      waypt_ = setpoint;
    }
  }

  if (obstacle_ && no_progress_rise_ && !too_low_ && !is_near_min_height_ ){
    waypt_.vector.z = pose_.pose.position.z + rise_factor_no_progress_;
    std::cout << "\033[1;34m No progress, increase height.\n \033[0m";
  }

  ROS_INFO("Selected waypoint: [%f, %f, %f].", waypt_.vector.x, waypt_.vector.y, waypt_.vector.z);

  getPathMsg();
}

// check that each point in the filtered point cloud is at least 0.5m away from the waypoint the UAV is flying to
bool LocalPlanner::checkForCollision() {
  std::clock_t start_time = std::clock();
  bool avoid = false;

  if(distance_to_closest_point_ < 0.5f) {
    avoid = true;
  }

  collision_time_.push_back((std::clock() - start_time) / (double(CLOCKS_PER_SEC / 1000)));
  return avoid;
}

void LocalPlanner::getMinFlightHeight() {
  int i = 0;
  double margin = 1;
  bool over_obstacle_old = over_obstacle_;
  double min_flight_height_old = min_flight_height_;
  over_obstacle_ = false;
  min_flight_height_ = -1000;
  double begin_rise_temp;

  for (std::vector<double>::iterator it = ground_heights_.begin(); it != ground_heights_.end(); ++it) {
    double flight_height = ground_heights_[i] + min_dist_to_ground_;
    double begin_rise;
    if (over_obstacle_old && flight_height > min_flight_height_old - 0.1 && flight_height < min_flight_height_old + 0.1) {
      begin_rise = begin_rise_;
    } else if (flight_height > pose_.pose.position.z) {
      double dist_diff = flight_height - pose_.pose.position.z;
      int e_max = floor(v_fov / 2);
      e_max = e_max - e_max % alpha_res;
      e_max = e_max + (alpha_res - e_max % alpha_res);  //[-80,+90]
      begin_rise = dist_diff / tan(e_max * PI / 180.0);
    } else {
      begin_rise = 0;
    }
    double xmin_begin_rise = 0;
    double xmax_begin_rise = begin_rise;
    double ymin_begin_rise = 0;
    double ymax_begin_rise = begin_rise;
    if (curr_vel_.twist.linear.x > 0) {
      xmin_begin_rise = begin_rise;
      xmax_begin_rise = 0;
    }
    if (curr_vel_.twist.linear.y > 0) {
      ymin_begin_rise = begin_rise;
      ymax_begin_rise = 0;
    }
    if (pose_.pose.position.x < ground_xmax_[i] + xmax_begin_rise && pose_.pose.position.x > ground_xmin_[i] - xmin_begin_rise && pose_.pose.position.y < ground_ymax_[i] + ymax_begin_rise
        && pose_.pose.position.y > ground_ymin_[i] - ymin_begin_rise) {
      if (flight_height > min_flight_height_) {
        min_flight_height_ = flight_height;
        over_obstacle_ = true;
        begin_rise_temp = begin_rise;
      }
    }

    i++;
  }
  begin_rise_ = begin_rise_temp;
  if (over_obstacle_) {
    std::cout << "\033[1;36m Minimal flight height: " << min_flight_height_ << "\n \033[0m";
  }
}

// if there isn't any obstacle in front of the UAV, increase cruising speed
void LocalPlanner::goFast(){

  if (withinGoalRadius()) {
    if (over_obstacle_ && (is_near_min_height_ || too_low_)) {
      ROS_INFO("Above Goal cannot go lower: Hoovering");
      waypt_.vector.x = goal_.x;
      waypt_.vector.y = goal_.y;
      if (pose_.pose.position.z > goal_.z) {
        waypt_.vector.z = pose_.pose.position.z;
      } else {
        waypt_.vector.z = goal_.z;
      }
    } else {
      ROS_INFO("Goal Reached: Hoovering");
      waypt_.vector.x = goal_.x;
      waypt_.vector.y = goal_.y;
      waypt_.vector.z = goal_.z;
    }
  } else {
    tf::Vector3 vec;
    vec.setX(goal_.x - pose_.pose.position.x);
    vec.setY(goal_.y - pose_.pose.position.y);
    vec.setZ(goal_.z - pose_.pose.position.z);
    double new_len = vec.length() < 1.0 ? vec.length() : speed_;

    //Prevent downward motion or move up if too close to ground
    if (use_ground_detection_) {
      vec.normalize();
      getMinFlightHeight();
      if (over_obstacle_ && pose_.pose.position.z <= min_flight_height_) {
        vec.setZ(0.3 * (min_flight_height_ - pose_.pose.position.z));
        std::cout << "\033[1;36m Go Fast: Flight altitude too low, rising.\n \033[0m";
      }
      if (over_obstacle_ && pose_.pose.position.z > min_flight_height_ && pose_.pose.position.z < min_flight_height_ + 0.5 && vec.getZ() < 0) {
        vec.setZ(0);
        std::cout << "\033[1;36m Go Fast: Preventing downward motion.\n \033[0m";
      }
    }

    vec.normalize();
    vec *= new_len;

    waypt_.vector.x = pose_.pose.position.x + vec.getX();
    waypt_.vector.y = pose_.pose.position.y + vec.getY();
    waypt_.vector.z = pose_.pose.position.z + vec.getZ();

    // fill direction as straight ahead
    geometry_msgs::Point p; p.x = 0; p.y = 90; p.z = 0;
    path_waypoints_.cells.push_back(p);

    //to keep track of forward movement update params also in go Fast mode
    updateCostParameters();

    //reset candidates for visualization
    initGridCells(&path_candidates_);
    initGridCells(&path_rejected_);
    initGridCells(&path_blocked_);
    initGridCells(&path_selected_);
    initGridCells(&path_ground_);

    ROS_INFO("Go fast selected direction: [%f, %f, %f].", vec.getX(), vec.getY(), vec.getZ());
    ROS_INFO("Go fast selected waypoint: [%f, %f, %f].", waypt_.vector.x, waypt_.vector.y, waypt_.vector.z);

    getPathMsg();
  }
}

void LocalPlanner::backOff() {

  tf::Vector3 vec;
  vec.setX(pose_.pose.position.x - back_off_point_.x);
  vec.setY(pose_.pose.position.y - back_off_point_.y);
  vec.setZ(0);
  vec.normalize();
  double new_len = speed_;
  vec *= new_len;

  waypt_.vector.x = pose_.pose.position.x + vec.getX();
  waypt_.vector.y = pose_.pose.position.y + vec.getY();
  waypt_.vector.z = pose_.pose.position.z + vec.getZ();

  double dist = distance3DCartesian(pose_.pose.position, back_off_point_);
  if (dist > 2.0) {
    back_off_ = false;
  }

  // fill direction as straight ahead
  geometry_msgs::Point p; p.x = 0; p.y = 90; p.z = 0;
  path_waypoints_.cells.push_back(p);

  std::cout<<"Distance: "<<dist<<"\n";
  ROS_INFO("Back off selected direction: [%f, %f, %f].", vec.getX(), vec.getY(), vec.getZ());
  ROS_INFO("Back off selected waypoint: [%f, %f, %f].", waypt_.vector.x, waypt_.vector.y, waypt_.vector.z);

  getPathMsg();
}

// check if the UAV has reached the goal set for the mission
bool LocalPlanner::withinGoalRadius(){
  geometry_msgs::Point a;
  a.x = std::abs(goal_.x - pose_.pose.position.x);
  a.y = std::abs(goal_.y - pose_.pose.position.y);
  a.z = std::abs(goal_.z - pose_.pose.position.z);
  float goal_acceptance_radius = 0.5f;

  if(a.x < goal_acceptance_radius && a.y < goal_acceptance_radius && a.z < goal_acceptance_radius){
    if(!reached_goal_){
      yaw_reached_goal_ = tf::getYaw(pose_.pose.orientation);
    }
    reached_goal_ = true;
    return true;
  } else if(over_obstacle_ && a.x < goal_acceptance_radius && a.y < goal_acceptance_radius){
    if(!reached_goal_){
      yaw_reached_goal_ = tf::getYaw(pose_.pose.orientation);
    }
    reached_goal_ = true;
    return true;
  }
  else
    return false;
}

geometry_msgs::PoseStamped LocalPlanner::createPoseMsg(geometry_msgs::Vector3Stamped waypt, double yaw) {
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.header.frame_id="/world";
  pose_msg.pose.position.x = waypt_.vector.x;
  pose_msg.pose.position.y = waypt_.vector.y;
  pose_msg.pose.position.z = waypt_.vector.z;
  pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
  return pose_msg;
}

// calculate the yaw for the next waypoint
double LocalPlanner::nextYaw(geometry_msgs::PoseStamped u, geometry_msgs::Vector3Stamped v, double last_yaw_) {
  double dx = v.vector.x - u.pose.position.x;
  double dy = v.vector.y - u.pose.position.y;

  if (round(dx) == 0 && round(dy) == 0) {
    return last_yaw_;   // Going up or down
  }

  if (reached_goal_){
    return yaw_reached_goal_;
  }

  return atan2(dy, dx);
}

// when taking off, first publish waypoints to reach the goal altitude
void LocalPlanner::reachGoalAltitudeFirst(){
  if (pose_.pose.position.z < (take_off_pose_.pose.position.z + goal_.z - 0.5)) {
      waypt_.vector.x = 0.0;
      waypt_.vector.y = 0.0;
      waypt_.vector.z = pose_.pose.position.z + 0.5;
  } else {
    reach_altitude_ = true;
    printf("Reached altitude %f, now going towards the goal_. \n\n",pose_.pose.position.z);
    getPathMsg();
  }
}

// smooth trajectory by liming the maximim accelleration possible
geometry_msgs::Vector3Stamped LocalPlanner::smoothWaypoint(){
  geometry_msgs::Vector3Stamped smooth_waypt;
  std::clock_t t = std::clock();
  float dt = (t - t_prev_) / (float)(CLOCKS_PER_SEC);
  dt = dt > 0.0f ? dt : 0.004f;
  t_prev_ = t;
  Eigen::Vector2f vel_xy(curr_vel_.twist.linear.x, curr_vel_.twist.linear.y);
  Eigen::Vector2f vel_waypt_xy((waypt_.vector.x - last_waypt_p_.pose.position.x) / dt, (waypt_.vector.y - last_waypt_p_.pose.position.y) / dt);
  Eigen::Vector2f vel_waypt_xy_prev((last_waypt_p_.pose.position.x - last_last_waypt_p_.pose.position.x) / dt, (last_waypt_p_.pose.position.y - last_last_waypt_p_.pose.position.y) / dt);
  Eigen::Vector2f acc_waypt_xy((vel_waypt_xy - vel_waypt_xy_prev) / dt);

  if (acc_waypt_xy.norm() > (acc_waypt_xy.norm() / 2.0f)) {
    vel_xy = (acc_waypt_xy.norm() / 2.0f) * acc_waypt_xy.normalized() * dt + vel_waypt_xy_prev;
  }

  float vel_waypt_z = (waypt_.vector.z - last_waypt_p_.pose.position.z) / dt;
  float max_acc_z, vel_z;
  float vel_waypt_z_prev = (last_waypt_p_.pose.position.z - last_last_waypt_p_.pose.position.z) / dt;
  float acc_waypt_z = (vel_waypt_z - vel_waypt_z_prev) / dt;

  max_acc_z = (acc_waypt_z < 0.0f) ? -(max_accel_z_) : (max_accel_z_);
  if (fabsf(acc_waypt_z) > fabsf(max_acc_z)) {
    vel_z = max_acc_z * dt + vel_waypt_z_prev;
  }

  smooth_waypt.vector.x = last_waypt_p_.pose.position.x + vel_xy(0) * dt;
  smooth_waypt.vector.y = last_waypt_p_.pose.position.y + vel_xy(1) * dt;
  smooth_waypt.vector.z = waypt_.vector.z;

  ROS_INFO("Smothed waypoint: [%f %f %f].", smooth_waypt.vector.x, smooth_waypt.vector.y, smooth_waypt.vector.z);
  return smooth_waypt;
}

// create the message that is sent to the UAV
void LocalPlanner::getPathMsg() {
  path_msg_.header.frame_id="/world";
  last_last_waypt_p_ = last_waypt_p_;
  last_waypt_p_ = waypt_p_;
  last_yaw_ = curr_yaw_;

  //first reach the altitude of the goal then start to move towards it (optional, comment out the entire if)
  if(!reach_altitude_){
    reachGoalAltitudeFirst();
  } else {
      if(!reached_goal_ && (pose_.pose.position.z > 1.5) && !stop_in_front_){
//        waypt_ = smoothWaypoint(); //Does not work with yaw only
      }
  }

  double new_yaw = nextYaw(pose_, waypt_, last_yaw_);
  only_yawed_ = false;
  //If the waypoint is not inside the FOV, only yaw and not move
  if(waypoint_outside_FOV_  && reach_altitude_ && !reached_goal_ && obstacle_ && !back_off_){
    waypt_.vector.x = pose_.pose.position.x;
    waypt_.vector.y = pose_.pose.position.y;
    waypt_.vector.z = pose_.pose.position.z;
    only_yawed_ = true;
  }


  waypt_p_ = createPoseMsg(waypt_, new_yaw);
  path_msg_.poses.push_back(waypt_p_);
  curr_yaw_ = new_yaw;
  position_old_ = pose_.pose.position;
  checkSpeed();
}

void LocalPlanner::printAlgorithmStatistics(){
  ROS_INFO("Current pose: [%f, %f, %f].", pose_.pose.position.x, pose_.pose.position.y, pose_.pose.position.z);
  ROS_INFO("Velocity: [%f, %f, %f], module: %f.", velocity_x_, velocity_y_, velocity_z_, velocity_mod_);

  if(log_data_to_txt_file_){
    logData();
  }

  if (withinGoalRadius()) {
    cv::Scalar mean, std;
    printf("----------------------------------- \n");
    cv::meanStdDev(algorithm_total_time, mean, std); printf("total mean %f std %f \n", mean[0], std[0]);
    cv::meanStdDev(cloud_time_, mean, std); printf("cloud mean %f std %f \n", mean[0], std[0]);
    cv::meanStdDev(polar_time_, mean, std); printf("polar mean %f std %f \n", mean[0], std[0]);
    cv::meanStdDev(free_time_, mean, std); printf("free mean %f std %f \n", mean[0], std[0]);
    cv::meanStdDev(cost_time_, mean, std); printf("cost mean %f std %f \n", mean[0], std[0]);
    cv::meanStdDev(collision_time_, mean, std); printf("collision mean %f std %f \n", mean[0], std[0]);
    printf("----------------------------------- \n");
  }
}

void LocalPlanner::checkSpeed(){
  if (hasSameYawAndAltitude(last_waypt_p_, waypt_p_) && !obstacleAhead()){
    speed_ = std::min(max_speed_, speed_ + 0.1);
  }
  else{
    speed_ = min_speed_;
  }
}

// check if two points have the same altitude and yaw
bool LocalPlanner::hasSameYawAndAltitude(geometry_msgs::PoseStamped msg1, geometry_msgs::PoseStamped msg2){
  return abs(msg1.pose.orientation.z) >= abs(0.9*msg2.pose.orientation.z) && abs(msg1.pose.orientation.z) <= abs(1.1*msg2.pose.orientation.z)
         && abs(msg1.pose.orientation.w) >= abs(0.9*msg2.pose.orientation.w) && abs(msg1.pose.orientation.w) <= abs(1.1*msg2.pose.orientation.w)
         && abs(msg1.pose.position.z) >= abs(0.9*msg2.pose.position.z) && abs(msg1.pose.position.z) <= abs(1.1*msg2.pose.position.z);

}


// stop in front of an obstacle at a distance defined by the variable keep_distance_
void LocalPlanner::stopInFrontObstacles(){

  if (first_brake_) {
    double braking_distance = fabsf(distance_to_closest_point_ - keep_distance_);
    Eigen::Vector2f pose_to_goal(goal_.x - pose_.pose.position.x, goal_.y - pose_.pose.position.y);
    goal_.x = pose_.pose.position.x + (braking_distance * pose_to_goal(0) / pose_to_goal.norm());
    goal_.y = pose_.pose.position.y + (braking_distance * pose_to_goal(1) / pose_to_goal.norm());
    first_brake_ = false;
  }
  ROS_INFO("New Stop Goal: [%.2f %.2f %.2f], obstacle distance %.2f. ", goal_.x, goal_.y, goal_.z, distance_to_closest_point_);
  goFast();
}

//Get the dimensions of the bounding box
void LocalPlanner::getBoundingBoxSize(double &min_x, double &max_x, double &min_y, double &max_y, double &min_z, double &max_z){
  min_x = min_box_x_;
  max_x = max_box_x_;
  min_y = min_box_y_;
  max_y = max_box_y_;
  min_z = min_box_z_;
  max_z = max_box_z_;
}

//Get the dimensions of the ground box
void LocalPlanner::getGroundBoxSize(double &min_x, double &max_x, double &min_y, double &max_y, double &min_z){
  min_x = min_groundbox_x_;
  max_x = max_groundbox_x_;
  min_y = min_groundbox_y_;
  max_y = max_groundbox_y_;
  min_z = min_groundbox_z_;
}

void LocalPlanner::getPosition(geometry_msgs::PoseStamped &pos){
  pos = pose_;
}

void LocalPlanner::getGoalPosition(geometry_msgs::Point  &goal){
  goal = goal_;
}

void LocalPlanner::getCloudsForVisualization(pcl::PointCloud<pcl::PointXYZ> &final_cloud, pcl::PointCloud<pcl::PointXYZ> &ground_cloud, pcl::PointCloud<pcl::PointXYZ> &reprojected_points){
  final_cloud = final_cloud_;
  ground_cloud = ground_cloud_;
  reprojected_points = reprojected_points_;
}

void LocalPlanner::getCandidateDataForVisualization(nav_msgs::GridCells &path_candidates, nav_msgs::GridCells &path_selected, nav_msgs::GridCells &path_rejected, nav_msgs::GridCells &path_blocked,
                                      nav_msgs::GridCells &path_ground){
  path_candidates = path_candidates_;
  path_selected = path_selected_;
  path_rejected = path_rejected_;
  path_blocked = path_blocked_;
  path_ground = path_ground_;
}

void LocalPlanner::getPathData(nav_msgs::Path &path_msg, geometry_msgs::PoseStamped &waypt_p){
  path_msg = path_msg_;
  waypt_p = waypt_p_;
}

void LocalPlanner::getGroundDataForVisualization(geometry_msgs::Point &closest_point_on_ground, geometry_msgs::Quaternion &ground_orientation){
  closest_point_on_ground = closest_point_on_ground_;
  ground_orientation = ground_orientation_;

}

void LocalPlanner::setCurrentVelocity(geometry_msgs::TwistStamped vel){
  curr_vel_ = vel;
}
