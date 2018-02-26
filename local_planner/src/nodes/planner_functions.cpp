#include "planner_functions.h"


// initialize GridCell message
void initGridCells(nav_msgs::GridCells *cell) {
  cell->cells.clear();
  cell->header.stamp = ros::Time::now();
  cell->header.frame_id = "/world";
  cell->cell_width = alpha_res;
  cell->cell_height = alpha_res;
  cell->cells = {};
}

//calculate sphere center from close points
void calculateSphere(geometry_msgs::Point &sphere_center, int &sphere_age, geometry_msgs::Point temp_centerpoint, int counter_sphere_points, double sphere_speed){
  if (counter_sphere_points > 50) {
    if(sphere_age < 10){
      tf::Vector3 vec;
      vec.setX(temp_centerpoint.x - sphere_center.x);
      vec.setY(temp_centerpoint.y - sphere_center.y);
      vec.setZ(temp_centerpoint.z - sphere_center.z);
      vec.normalize();
      double new_len = sphere_speed;
      vec *= new_len;
      sphere_center.x = sphere_center.x + vec.getX();
      sphere_center.y = sphere_center.y + vec.getY();
      sphere_center.z = sphere_center.z + vec.getZ();
    } else {
      sphere_center.x = temp_centerpoint.x;
      sphere_center.y = temp_centerpoint.y;
      sphere_center.z = temp_centerpoint.z;
    }
    sphere_age = 0;
  }else{
    sphere_age ++;
  }
}

//adapt histogram safety margin around blocked cells to distance of pointcloud
double adaptSafetyMarginHistogram(double dist_to_closest_point, double cloud_size, double min_cloud_size) {
  double safety_margin = 25;

  //increase safety radius if too close to the wall
  if (dist_to_closest_point < 1.7 && cloud_size > min_cloud_size) {
    safety_margin = 25 + 2 * alpha_res;
  }
  return safety_margin;
}

// trim the point cloud so that only points inside the bounding box are considered and
void filterPointCloud(pcl::PointCloud<pcl::PointXYZ> &cropped_cloud, geometry_msgs::Point &closest_point, geometry_msgs::Point &temp_sphere_center, double &distance_to_closest_point, int &counter_backoff, int &counter_sphere,
                                    pcl::PointCloud<pcl::PointXYZ> complete_cloud, double min_cloud_size, double min_dist_backoff, double sphere_radius, Box histogram_box, geometry_msgs::Point position) {
  double min_realsense_dist = 0.2;
  std::clock_t start_time = std::clock();
  pcl::PointCloud<pcl::PointXYZ>::iterator pcl_it;
  cropped_cloud.points.clear();
  cropped_cloud.width = 0;
  distance_to_closest_point = inf;
  float distance;
  counter_backoff = 0;
  counter_sphere = 0;

  for (pcl_it = complete_cloud.begin(); pcl_it != complete_cloud.end(); ++pcl_it) {
    // Check if the point is invalid
    if (!std::isnan(pcl_it->x) && !std::isnan(pcl_it->y) && !std::isnan(pcl_it->z)) {
      if (histogram_box.isPointWithin(pcl_it->x, pcl_it->y, pcl_it->z)) {
        distance = computeL2Dist(position, pcl_it);
        if (distance > min_realsense_dist) {
          cropped_cloud.points.push_back(pcl::PointXYZ(pcl_it->x, pcl_it->y, pcl_it->z));
          if (distance < distance_to_closest_point) {
            distance_to_closest_point = distance;
            closest_point.x = pcl_it->x;
            closest_point.y = pcl_it->y;
            closest_point.z = pcl_it->z;
          }
          if (distance < min_dist_backoff) {
            counter_backoff++;
          }
          if (distance < sphere_radius + 1.5) {
            counter_sphere++;
            temp_sphere_center.x += pcl_it->x;
            temp_sphere_center.y += pcl_it->y;
            temp_sphere_center.z += pcl_it->z;
          }
        }
      }
    }
  }

  temp_sphere_center.x = temp_sphere_center.x / counter_sphere;
  temp_sphere_center.y = temp_sphere_center.y / counter_sphere;
  temp_sphere_center.z = temp_sphere_center.z / counter_sphere;

  cropped_cloud.header.stamp = complete_cloud.header.stamp;
  cropped_cloud.header.frame_id = complete_cloud.header.frame_id;
  cropped_cloud.height = 1;
  if (cropped_cloud.points.size() <= min_cloud_size) {
    cropped_cloud.points.clear();
    cropped_cloud.width = 0;
  }
}

// Calculate FOV. Azimuth angle is wrapped, elevation is not!
void calculateFOV(std::vector<int> &z_FOV_idx, int &e_FOV_min, int &e_FOV_max, double yaw, double pitch) {

  double z_FOV_max = std::round((-yaw * 180.0 / PI + h_fov / 2.0 + 270.0) / alpha_res) - 1;
  double z_FOV_min = std::round((-yaw * 180.0 / PI - h_fov / 2.0 + 270.0) / alpha_res) - 1;
  e_FOV_max = std::round((-pitch * 180.0 / PI + v_fov / 2.0 + 90.0) / alpha_res) - 1;
  e_FOV_min = std::round((-pitch * 180.0 / PI - v_fov / 2.0 + 90.0) / alpha_res) - 1;

  if (z_FOV_max >= grid_length_z && z_FOV_min >= grid_length_z) {
    z_FOV_max -= grid_length_z;
    z_FOV_min -= grid_length_z;
  }
  if (z_FOV_max < 0 && z_FOV_min < 0) {
    z_FOV_max += grid_length_z;
    z_FOV_min += grid_length_z;
  }

  z_FOV_idx.clear();
  if (z_FOV_max >= grid_length_z && z_FOV_min < grid_length_z) {
    for (int i = 0; i < z_FOV_max - grid_length_z; i++) {
      z_FOV_idx.push_back(i);
    }
    for (int i = z_FOV_min; i < grid_length_z; i++) {
      z_FOV_idx.push_back(i);
    }
  } else if (z_FOV_min < 0 && z_FOV_max >= 0) {
    for (int i = 0; i < z_FOV_max; i++) {
      z_FOV_idx.push_back(i);
    }
    for (int i = z_FOV_min + grid_length_z; i < grid_length_z; i++) {
      z_FOV_idx.push_back(i);
    }
  } else {
    for (int i = z_FOV_min; i < z_FOV_max; i++) {
      z_FOV_idx.push_back(i);
    }
  }
}

//Build histogram estimate from reprojected points
void propagateHistogram(Histogram &polar_histogram_est, pcl::PointCloud<pcl::PointXYZ> reprojected_points, std::vector<double> reprojected_points_age, std::vector<double> reprojected_points_dist, geometry_msgs::PoseStamped position) {

  for (int i = 0; i < reprojected_points.points.size(); i++) {
    int e_angle = elevationAnglefromCartesian(reprojected_points.points[i].x, reprojected_points.points[i].y, reprojected_points.points[i].z, position.pose.position);
    int z_angle = azimuthAnglefromCartesian(reprojected_points.points[i].x, reprojected_points.points[i].y, reprojected_points.points[i].z, position.pose.position);

    int e_ind = elevationAngletoIndex(e_angle, 2*alpha_res);
    int z_ind = azimuthAngletoIndex(z_angle, 2*alpha_res);

    polar_histogram_est.set_bin(e_ind, z_ind, polar_histogram_est.get_bin(e_ind, z_ind) + 0.25);
    polar_histogram_est.set_age(e_ind, z_ind, polar_histogram_est.get_age(e_ind, z_ind) + 0.25 * reprojected_points_age[i]);
    polar_histogram_est.set_dist(e_ind, z_ind, polar_histogram_est.get_dist(e_ind, z_ind) + 0.25 * reprojected_points_dist[i]);
  }

  for (int e = 0; e < grid_length_e / 2; e++) {
    for (int z = 0; z < grid_length_z / 2; z++) {
      if (polar_histogram_est.get_bin(e, z) >= min_bin) {
        polar_histogram_est.set_dist(e, z, polar_histogram_est.get_dist(e, z) / polar_histogram_est.get_bin(e, z));
        polar_histogram_est.set_age(e, z, polar_histogram_est.get_age(e, z) / polar_histogram_est.get_bin(e, z));
        polar_histogram_est.set_bin(e, z, 1);
      } else {
        polar_histogram_est.set_dist(e, z, 0);
        polar_histogram_est.set_age(e, z, 0);
        polar_histogram_est.set_bin(e, z, 0);
      }
    }
  }

  //Upsample propagated histogram
  polar_histogram_est.upsample();
}

//Generate new histogram from pointcloud
void generateNewHistogram(Histogram &polar_histogram, pcl::PointCloud<pcl::PointXYZ> cropped_cloud, geometry_msgs::PoseStamped position) {
  pcl::PointCloud<pcl::PointXYZ>::const_iterator it;
  geometry_msgs::Point temp;
  double dist;

  for (it = cropped_cloud.begin(); it != cropped_cloud.end(); ++it) {
    temp.x = it->x;
    temp.y = it->y;
    temp.z = it->z;
    dist = distance3DCartesian(position.pose.position, temp);

    int e_angle = elevationAnglefromCartesian(temp.x, temp.y, temp.z, position.pose.position);
    int z_angle = azimuthAnglefromCartesian(temp.x, temp.y, temp.z, position.pose.position);

    int e_ind = elevationAngletoIndex(e_angle, alpha_res);
    int z_ind = azimuthAngletoIndex(z_angle, alpha_res);

    polar_histogram.set_bin(e_ind, z_ind, polar_histogram.get_bin(e_ind, z_ind) + 1);
    polar_histogram.set_dist(e_ind, z_ind, polar_histogram.get_dist(e_ind, z_ind) + dist);
  }

  //Normalize and get mean in distance bins
  for (int e = 0; e < grid_length_e; e++) {
    for (int z = 0; z < grid_length_z; z++) {
      if (polar_histogram.get_bin(e, z) > 0) {
        polar_histogram.set_dist(e, z, polar_histogram.get_dist(e, z) / polar_histogram.get_bin(e, z));
        polar_histogram.set_bin(e, z, 1);
      }
    }
  }
}


//Combine propagated histogram and new histogram to the final binary histogram
void combinedHistogram(bool &hist_empty, Histogram &new_hist, Histogram propagated_hist, bool waypoint_outside_FOV, std::vector<int> z_FOV_idx, int e_FOV_min, int e_FOV_max) {
  hist_empty = true;
  for (int e = 0; e < grid_length_e; e++) {
    for (int z = 0; z < grid_length_z; z++) {
      if (std::find(z_FOV_idx.begin(), z_FOV_idx.end(), z) != z_FOV_idx.end() && e > e_FOV_min && e < e_FOV_max) {  //inside FOV
        if (new_hist.get_bin(e, z) > 0) {
          new_hist.set_age(e, z, 1);
          hist_empty = false;
        }
      } else {
        if (propagated_hist.get_bin(e, z) > 0) {
          if (waypoint_outside_FOV) {
            new_hist.set_age(e, z, propagated_hist.get_age(e, z));
          } else {
            new_hist.set_age(e, z, propagated_hist.get_age(e, z) + 1);
          }
          hist_empty = false;
        }
        if (new_hist.get_bin(e, z) > 0) {
          new_hist.set_age(e, z, 1);
          hist_empty = false;
        }
        if (propagated_hist.get_bin(e, z) > 0 && new_hist.get_bin(e, z) == 0) {
          new_hist.set_bin(e, z, propagated_hist.get_bin(e, z));
          new_hist.set_dist(e, z, propagated_hist.get_dist(e, z));
        }
      }
    }
  }
}

//costfunction for every free histogram cell
double costFunction(int e, int z, nav_msgs::GridCells path_waypoints, geometry_msgs::Point goal, geometry_msgs::PoseStamped position, geometry_msgs::Point position_old, double goal_cost_param, double smooth_cost_param,
                    double height_change_cost_param_adapted, double height_change_cost_param, bool only_yawed) {

  double cost;
  int waypoint_index = path_waypoints.cells.size();

  double dist = distance3DCartesian(position.pose.position, goal);
  double dist_old = distance3DCartesian(position_old, goal);
  geometry_msgs::Point candidate_goal = fromPolarToCartesian(e, z, dist, position.pose.position);
  geometry_msgs::Point old_candidate_goal = fromPolarToCartesian(path_waypoints.cells[waypoint_index - 1].x, path_waypoints.cells[waypoint_index - 1].y, dist_old, position_old);
  double yaw_cost = goal_cost_param * sqrt((goal.x - candidate_goal.x) * (goal.x - candidate_goal.x) + (goal.y - candidate_goal.y) * (goal.y - candidate_goal.y));
  double pitch_cost_up = 0;
  double pitch_cost_down = 0;
  if (candidate_goal.z > goal.z) {
    pitch_cost_up = goal_cost_param * sqrt((goal.z - candidate_goal.z) * (goal.z - candidate_goal.z));
  } else {
    pitch_cost_down = goal_cost_param * sqrt((goal.z - candidate_goal.z) * (goal.z - candidate_goal.z));
  }

  double yaw_cost_smooth = smooth_cost_param * sqrt((old_candidate_goal.x - candidate_goal.x) * (old_candidate_goal.x - candidate_goal.x) + (old_candidate_goal.y - candidate_goal.y) * (old_candidate_goal.y - candidate_goal.y));
  double pitch_cost_smooth = smooth_cost_param * sqrt((old_candidate_goal.z - candidate_goal.z) * (old_candidate_goal.z - candidate_goal.z));

  if (!only_yawed) {
    cost = yaw_cost + height_change_cost_param_adapted * pitch_cost_up + height_change_cost_param * pitch_cost_down + yaw_cost_smooth + pitch_cost_smooth;
  } else {
    cost = yaw_cost + height_change_cost_param_adapted * pitch_cost_up + height_change_cost_param * pitch_cost_down + 0.1 * yaw_cost_smooth + 0.1 * pitch_cost_smooth;
  }

  return cost;
}

// search for free directions in the 2D polar histogram with a moving window approach
void findFreeDirections(Histogram histogram, double safety_radius, nav_msgs::GridCells &path_candidates, nav_msgs::GridCells &path_selected, nav_msgs::GridCells &path_rejected,
                                      nav_msgs::GridCells &path_blocked, nav_msgs::GridCells &path_ground, nav_msgs::GridCells path_waypoints, std::vector<float> &cost_path_candidates,
                                      geometry_msgs::Point goal, geometry_msgs::PoseStamped position, geometry_msgs::Point position_old, double goal_cost_param,
                                      double smooth_cost_param, double height_change_cost_param_adapted, double height_change_cost_param, int e_min_idx, bool over_obstacle, bool only_yawed, int resolution_alpha) {
  std::clock_t start_time = std::clock();
  int n = floor(safety_radius / resolution_alpha);  //safety radius
  int z_dim = 360 / resolution_alpha;
  int e_dim = 180 / resolution_alpha;
  int a = 0, b = 0;
  bool free = true;
  bool corner = false;
  bool height_reject = false;
  geometry_msgs::Point p;
  cost_path_candidates.clear();

  initGridCells(&path_candidates);
  initGridCells(&path_rejected);
  initGridCells(&path_blocked);
  initGridCells(&path_selected);
  initGridCells(&path_ground);

  //determine which bins are candidates
  for (int e = 0; e < e_dim; e++) {
    for (int z = 0; z < z_dim; z++) {
      for (int i = (e - n); i <= (e + n); i++) {
        for (int j = (z - n); j <= (z + n); j++) {

          free = true;
          corner = false;
          height_reject = false;

          // Elevation index < 0
          if(i < 0 && j >= 0 && j < z_dim) {
            a = -i;
            b = z_dim - j - 1;
          }
          // Azimuth index < 0
          else if(j < 0 && i >= 0 && i < e_dim) {
            b = j+z_dim;
          }
          // Elevation index > grid_length_e
          else if(i >= e_dim && j >= 0 && j < z_dim) {
            a = e_dim - (i % (e_dim - 1));
            b = z_dim - j - 1;
          }
          // Azimuth index > grid_length_z
          else if(j >= z_dim && i >= 0 && i < e_dim) {
            b = j - z_dim;
          }
          // Elevation and Azimuth index both within histogram
          else if( i >= 0 && i < e_dim && j >= 0 && j < z_dim) {
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
            if(histogram.get_bin(a,b) != 0) {
              free = false;
              break;
            }
          }
        }

        if(!free)
          break;
      }

      //reject points which lead the drone too close to the ground
      if (over_obstacle) {
        if (e <= e_min_idx) {
          height_reject = true;
        }
      }

      if (free && !height_reject) {
        p.x = elevationIndexToAngle(e, resolution_alpha);
        p.y = azimuthIndexToAngle(z, resolution_alpha);
        p.z = 0;
        path_candidates.cells.push_back(p);
        double cost = costFunction((int) p.x, (int) p.y, path_waypoints, goal, position, position_old, goal_cost_param, smooth_cost_param, height_change_cost_param_adapted, height_change_cost_param, only_yawed);
        cost_path_candidates.push_back(cost);
      } else if (!free && histogram.get_bin(e, z) != 0 && !height_reject) {
        p.x = elevationIndexToAngle(e, resolution_alpha);
        p.y = azimuthIndexToAngle(z, resolution_alpha);
        p.z = 0;
        path_rejected.cells.push_back(p);
      } else if (height_reject) {
        p.x = elevationIndexToAngle(e, resolution_alpha);
        p.y = azimuthIndexToAngle(z, resolution_alpha);
        p.z = 0;
        path_ground.cells.push_back(p);
      } else {
        p.x = elevationIndexToAngle(e, resolution_alpha);
        p.y = azimuthIndexToAngle(z, resolution_alpha);
        p.z = 0;
        path_blocked.cells.push_back(p);
      }
    }
  }
}

// calculate the free direction which has the smallest cost for the UAV to travel to
bool calculateCostMap(std::vector<float> cost_path_candidates, std::vector<int> &cost_idx_sorted) {
  if(cost_path_candidates.empty()){
    std::cout << "\033[1;31mbold Empty candidates vector!--------------------------------\033[0m\n";
    return 1;
  }else{
    cv::sortIdx(cost_path_candidates, cost_idx_sorted, CV_SORT_EVERY_ROW + CV_SORT_ASCENDING);
    return 0;
  }
}

void printHistogram(Histogram hist, std::vector<int> z_FOV_idx, int e_FOV_min, int e_FOV_max, int e_chosen, int z_chosen, double resolution) {
  int z_dim = 360 / resolution;
  int e_dim = 180 / resolution;

  for (int e_ind = 0; e_ind < e_dim; e_ind++) {
    for (int z_ind = 0; z_ind < z_dim; z_ind++) {
      if (e_chosen == e_ind && z_chosen == z_ind) {
        std::cout << "\033[1;31m" << hist.get_bin(e_ind, z_ind) << " \033[0m";
      } else if (std::find(z_FOV_idx.begin(), z_FOV_idx.end(), z_ind) != z_FOV_idx.end() && e_ind > e_FOV_min && e_ind < e_FOV_max) {
        std::cout << "\033[1;32m" << hist.get_bin(e_ind, z_ind) << " \033[0m";
      } else {
        std::cout << hist.get_bin(e_ind, z_ind) << " ";
      }
    }
    std::cout << "\n";
  }
  std::cout << "--------------------------------------\n";
}


