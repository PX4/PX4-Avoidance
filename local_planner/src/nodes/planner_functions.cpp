#include "planner_functions.h"

#include "common.h"

#include <ros/console.h>

#include <numeric>

namespace avoidance {

// initialize GridCell message
void initGridCells(nav_msgs::GridCells& cell) {
  cell.cells.clear();
  cell.header.stamp = ros::Time::now();
  cell.header.frame_id = "/local_origin";
  cell.cell_width = ALPHA_RES;
  cell.cell_height = ALPHA_RES;
  cell.cells = {};
}

// adapt histogram safety margin around blocked cells to distance of pointcloud
double adaptSafetyMarginHistogram(double dist_to_closest_point,
                                  double cloud_size, double min_cloud_size) {
  double safety_margin = 25;

  // increase safety radius if too close to the wall
  if (dist_to_closest_point < 1.7 && cloud_size > min_cloud_size) {
    safety_margin = 25 + 2 * ALPHA_RES;
  }
  return safety_margin;
}

// trim the point cloud so that only points inside the bounding box are
// considered
void filterPointCloud(
    pcl::PointCloud<pcl::PointXYZ>& cropped_cloud,
    Eigen::Vector3f& closest_point, double& distance_to_closest_point,
    int& counter_backoff,
    const std::vector<pcl::PointCloud<pcl::PointXYZ>>& complete_cloud,
    double min_cloud_size, double min_dist_backoff, Box histogram_box,
    const Eigen::Vector3f& position, double min_realsense_dist) {
  cropped_cloud.points.clear();
  cropped_cloud.width = 0;
  distance_to_closest_point = HUGE_VAL;
  float distance;
  counter_backoff = 0;

  for (const auto& cloud : complete_cloud) {
    for (const pcl::PointXYZ& xyz : cloud) {
      // Check if the point is invalid
      if (!std::isnan(xyz.x) && !std::isnan(xyz.y) && !std::isnan(xyz.z)) {
        if (histogram_box.isPointWithinBox(xyz.x, xyz.y, xyz.z)) {
          distance = (position - toEigen(xyz)).norm();
          if (distance > min_realsense_dist &&
              distance < histogram_box.radius_) {
            cropped_cloud.points.push_back(pcl::PointXYZ(xyz.x, xyz.y, xyz.z));
            if (distance < distance_to_closest_point) {
              distance_to_closest_point = distance;
              closest_point = toEigen(xyz);
            }
            if (distance < min_dist_backoff) {
              counter_backoff++;
            }
          }
        }
      }
    }
  }

  cropped_cloud.header.stamp = complete_cloud[0].header.stamp;
  cropped_cloud.header.frame_id = complete_cloud[0].header.frame_id;
  cropped_cloud.height = 1;
  cropped_cloud.width = cropped_cloud.points.size();
  if (cropped_cloud.points.size() <= min_cloud_size) {
    cropped_cloud.points.clear();
    cropped_cloud.width = 0;
  }
}

// Calculate FOV. Azimuth angle is wrapped, elevation is not!
void calculateFOV(double h_fov, double v_fov, std::vector<int>& z_FOV_idx,
                  int& e_FOV_min, int& e_FOV_max, double yaw, double pitch) {
  double z_FOV_max =
      std::round((-yaw * 180.0 / M_PI + h_fov / 2.0 + 270.0) / ALPHA_RES) - 1;
  double z_FOV_min =
      std::round((-yaw * 180.0 / M_PI - h_fov / 2.0 + 270.0) / ALPHA_RES) - 1;
  e_FOV_max =
      std::round((-pitch * 180.0 / M_PI + v_fov / 2.0 + 90.0) / ALPHA_RES) - 1;
  e_FOV_min =
      std::round((-pitch * 180.0 / M_PI - v_fov / 2.0 + 90.0) / ALPHA_RES) - 1;

  if (z_FOV_max >= GRID_LENGTH_Z && z_FOV_min >= GRID_LENGTH_Z) {
    z_FOV_max -= GRID_LENGTH_Z;
    z_FOV_min -= GRID_LENGTH_Z;
  }
  if (z_FOV_max < 0 && z_FOV_min < 0) {
    z_FOV_max += GRID_LENGTH_Z;
    z_FOV_min += GRID_LENGTH_Z;
  }

  z_FOV_idx.clear();
  if (z_FOV_max >= GRID_LENGTH_Z && z_FOV_min < GRID_LENGTH_Z) {
    for (int i = 0; i < z_FOV_max - GRID_LENGTH_Z; i++) {
      z_FOV_idx.push_back(i);
    }
    for (int i = z_FOV_min; i < GRID_LENGTH_Z; i++) {
      z_FOV_idx.push_back(i);
    }
  } else if (z_FOV_min < 0 && z_FOV_max >= 0) {
    for (int i = 0; i < z_FOV_max; i++) {
      z_FOV_idx.push_back(i);
    }
    for (int i = z_FOV_min + GRID_LENGTH_Z; i < GRID_LENGTH_Z; i++) {
      z_FOV_idx.push_back(i);
    }
  } else {
    for (int i = z_FOV_min; i < z_FOV_max; i++) {
      z_FOV_idx.push_back(i);
    }
  }
}

// Build histogram estimate from reprojected points
void propagateHistogram(
    Histogram& polar_histogram_est,
    const pcl::PointCloud<pcl::PointXYZ>& reprojected_points,
    const std::vector<double>& reprojected_points_age,
    const std::vector<double>& reprojected_points_dist,
    const geometry_msgs::PoseStamped& position) {
  for (size_t i = 0; i < reprojected_points.points.size(); i++) {
    PolarPoint p_pol = cartesianToPolar(toEigen(reprojected_points.points[i]),
                                        toEigen(position.pose.position));
    Eigen::Vector2i p_ind = polarToHistogramIndex(p_pol, 2 * ALPHA_RES);

    polar_histogram_est.set_bin(
        p_ind.y(), p_ind.x(),
        polar_histogram_est.get_bin(p_ind.y(), p_ind.x()) + 0.25);
    polar_histogram_est.set_age(
        p_ind.y(), p_ind.x(),
        polar_histogram_est.get_age(p_ind.y(), p_ind.x()) +
            0.25 * reprojected_points_age[i]);
    polar_histogram_est.set_dist(
        p_ind.y(), p_ind.x(),
        polar_histogram_est.get_dist(p_ind.y(), p_ind.x()) +
            0.25 * reprojected_points_dist[i]);
  }

  for (int e = 0; e < GRID_LENGTH_E / 2; e++) {
    for (int z = 0; z < GRID_LENGTH_Z / 2; z++) {
      if (polar_histogram_est.get_bin(e, z) >= 1.5) {
        polar_histogram_est.set_dist(e, z,
                                     polar_histogram_est.get_dist(e, z) /
                                         polar_histogram_est.get_bin(e, z));
        polar_histogram_est.set_age(e, z,
                                    polar_histogram_est.get_age(e, z) /
                                        polar_histogram_est.get_bin(e, z));
        polar_histogram_est.set_bin(e, z, 1);
      } else {
        polar_histogram_est.set_dist(e, z, 0);
        polar_histogram_est.set_age(e, z, 0);
        polar_histogram_est.set_bin(e, z, 0);
      }
    }
  }

  // Upsample propagated histogram
  polar_histogram_est.upsample();
}

// Generate new histogram from pointcloud
void generateNewHistogram(Histogram& polar_histogram,
                          const pcl::PointCloud<pcl::PointXYZ>& cropped_cloud,
                          geometry_msgs::PoseStamped position) {
  for (auto xyz : cropped_cloud) {
    Eigen::Vector3f p = toEigen(xyz);
    float dist = (p - toEigen(position.pose.position)).norm();
    PolarPoint p_pol = cartesianToPolar(p, toEigen(position.pose.position));
    Eigen::Vector2i p_ind = polarToHistogramIndex(p_pol, ALPHA_RES);

    polar_histogram.set_bin(p_ind.y(), p_ind.x(),
                            polar_histogram.get_bin(p_ind.y(), p_ind.x()) + 1);
    polar_histogram.set_dist(
        p_ind.y(), p_ind.x(),
        polar_histogram.get_dist(p_ind.y(), p_ind.x()) + dist);
  }

  // Normalize and get mean in distance bins
  for (int e = 0; e < GRID_LENGTH_E; e++) {
    for (int z = 0; z < GRID_LENGTH_Z; z++) {
      if (polar_histogram.get_bin(e, z) > 0) {
        polar_histogram.set_dist(e, z, polar_histogram.get_dist(e, z) /
                                           polar_histogram.get_bin(e, z));
        polar_histogram.set_bin(e, z, 1);
      }
    }
  }
}

// Combine propagated histogram and new histogram to the final binary histogram
void combinedHistogram(bool& hist_empty, Histogram& new_hist,
                       const Histogram& propagated_hist,
                       bool waypoint_outside_FOV,
                       const std::vector<int>& z_FOV_idx, int e_FOV_min,
                       int e_FOV_max) {
  hist_empty = true;
  for (int e = 0; e < GRID_LENGTH_E; e++) {
    for (int z = 0; z < GRID_LENGTH_Z; z++) {
      if (std::find(z_FOV_idx.begin(), z_FOV_idx.end(), z) != z_FOV_idx.end() &&
          e > e_FOV_min && e < e_FOV_max) {  // inside FOV
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

// costfunction for every free histogram cell
double costFunction(PolarPoint p_pol, nav_msgs::GridCells& path_waypoints,
                    const Eigen::Vector3f& goal,
                    const Eigen::Vector3f& position,
                    const Eigen::Vector3f& position_old, double goal_cost_param,
                    double smooth_cost_param,
                    double height_change_cost_param_adapted,
                    double height_change_cost_param, bool only_yawed) {
  double cost;
  int waypoint_index = path_waypoints.cells.size() - 1;
  if (waypoint_index < 0) {
    waypoint_index = 0;
    path_waypoints.cells.push_back(toPoint(position_old));
  }

  float dist = (position - goal).norm();
  float dist_old = (position_old - goal).norm();
  p_pol.r = dist;
  Eigen::Vector3f candidate_goal = polarToCartesian(p_pol, toPoint(position));
  PolarPoint p_pol_old(path_waypoints.cells[waypoint_index].x,
                       path_waypoints.cells[waypoint_index].y, dist_old);

  Eigen::Vector3f old_candidate_goal =
      polarToCartesian(p_pol_old, toPoint(position_old));
  double yaw_cost = goal_cost_param *
                    (goal.topRows<2>() - candidate_goal.topRows<2>()).norm();

  double pitch_cost_up = 0.0;
  double pitch_cost_down = 0.0;
  if (candidate_goal.z() > goal.z()) {
    pitch_cost_up = goal_cost_param * std::abs(goal.z() - candidate_goal.z());
  } else {
    pitch_cost_down = goal_cost_param * std::abs(goal.z() - candidate_goal.z());
  }

  double yaw_cost_smooth =
      smooth_cost_param *
      (old_candidate_goal.topRows<2>() - candidate_goal.topRows<2>()).norm();

  double pitch_cost_smooth =
      smooth_cost_param * std::abs(old_candidate_goal.z() - candidate_goal.z());

  if (!only_yawed) {
    cost = yaw_cost + height_change_cost_param_adapted * pitch_cost_up +
           height_change_cost_param * pitch_cost_down + yaw_cost_smooth +
           pitch_cost_smooth;
  } else {
    cost = yaw_cost + height_change_cost_param_adapted * pitch_cost_up +
           height_change_cost_param * pitch_cost_down + 0.5 * yaw_cost_smooth +
           0.5 * pitch_cost_smooth;
  }

  return cost;
}

void compressHistogramElevation(Histogram& new_hist,
                                const Histogram& input_hist) {
  float vertical_FOV_range_sensor = 20.0;
  PolarPoint p_pol_lower(-1 * vertical_FOV_range_sensor / 2.0, 0.0f, 0.0f);
  PolarPoint p_pol_upper(vertical_FOV_range_sensor / 2.0, 0.0f, 0.0f);
  Eigen::Vector2i p_ind_lower = polarToHistogramIndex(p_pol_lower, ALPHA_RES);
  Eigen::Vector2i p_ind_upper = polarToHistogramIndex(p_pol_upper, ALPHA_RES);

  for (int e = p_ind_lower.y(); e <= p_ind_upper.y(); e++) {
    for (int z = 0; z < GRID_LENGTH_Z; z++) {
      if (input_hist.get_bin(e, z) > 0) {
        new_hist.set_bin(0, z, new_hist.get_bin(0, z) + 1);
        if (input_hist.get_dist(e, z) < new_hist.get_dist(0, z) ||
            (new_hist.get_dist(0, z) == 0.0))
          new_hist.set_dist(0, z, input_hist.get_dist(e, z));
      }
    }
  }
}

// search for free directions in the 2D polar histogram with a moving window
// approach
void findFreeDirections(
    const Histogram& histogram, double safety_radius,
    nav_msgs::GridCells& path_candidates, nav_msgs::GridCells& path_selected,
    nav_msgs::GridCells& path_rejected, nav_msgs::GridCells& path_blocked,
    nav_msgs::GridCells& path_waypoints,
    std::vector<float>& cost_path_candidates, const Eigen::Vector3f& goal,
    const Eigen::Vector3f& position, const Eigen::Vector3f& position_old,
    double goal_cost_param, double smooth_cost_param,
    double height_change_cost_param_adapted, double height_change_cost_param,
    bool only_yawed, int resolution_alpha) {
  int n = floor(safety_radius / resolution_alpha);  // safety radius
  int z_dim = 360 / resolution_alpha;
  int e_dim = 180 / resolution_alpha;
  int a = 0, b = 0;
  bool free = true;
  bool corner = false;
  geometry_msgs::Point p;
  cost_path_candidates.clear();

  initGridCells(path_candidates);
  initGridCells(path_rejected);
  initGridCells(path_blocked);
  initGridCells(path_selected);

  // determine which bins are candidates
  for (int e = 0; e < e_dim; e++) {
    for (int z = 0; z < z_dim; z++) {
      for (int i = (e - n); i <= (e + n); i++) {
        for (int j = (z - n); j <= (z + n); j++) {
          free = true;
          corner = false;

          // Elevation index < 0
          if (i < 0 && j >= 0 && j < z_dim) {
            a = 0;
            b = j + (180 / resolution_alpha) - 1;
            if (b >= z_dim) {
              b = b % (z_dim - 1);
            }
          }
          // Azimuth index < 0
          else if (j < 0 && i >= 0 && i < e_dim) {
            b = j + z_dim;
            a = i;
          }
          // Elevation index > e_dim
          else if (i >= e_dim && j >= 0 && j < z_dim) {
            a = e_dim - (i % (e_dim - 1));
            b = j + (180 / resolution_alpha) - 1;
            if (b >= z_dim) {
              b = b % (z_dim - 1);
            };
          }
          // Azimuth index > z_dim
          else if (j >= z_dim && i >= 0 && i < e_dim) {
            b = j - z_dim;
          }
          // Elevation and Azimuth index both within histogram
          else if (i >= 0 && i < e_dim && j >= 0 && j < z_dim) {
            a = i;
            b = j;
          }
          // Elevation and azimuth index both < 0 OR elevation index >
          // e_dim and azimuth index <> z_dim OR elevation index < 0 and
          // azimuth index > z_dim OR elevation index > e_dim and azimuth index
          // < 0. These cells are not part of the polar histogram.
          else {
            corner = true;
          }

          if (!corner) {
            if (histogram.get_bin(a, b) != 0) {
              free = false;
              break;
            }
          }
        }

        if (!free) break;
      }

      if (free) {
        PolarPoint p_pol = histogramIndexToPolar(e, z, resolution_alpha, 0.0);
        p.x = p_pol.e;
        p.y = p_pol.z;
        p.z = p_pol.r;
        path_candidates.cells.push_back(p);
        double cost =
            costFunction(p_pol, path_waypoints, goal, position, position_old,
                         goal_cost_param, smooth_cost_param,
                         height_change_cost_param_adapted,
                         height_change_cost_param, only_yawed);
        cost_path_candidates.push_back(cost);
      } else if (!free && histogram.get_bin(e, z) != 0) {
        PolarPoint p_pol = histogramIndexToPolar(e, z, resolution_alpha, 0.0);
        p.x = p_pol.e;
        p.y = p_pol.z;
        p.z = p_pol.r;
        path_rejected.cells.push_back(p);
      } else {
        PolarPoint p_pol = histogramIndexToPolar(e, z, resolution_alpha, 0.0);
        p.x = p_pol.e;
        p.y = p_pol.z;
        p.z = p_pol.r;
        path_blocked.cells.push_back(p);
      }
    }
  }
}

// calculate the free direction which has the smallest cost for the UAV to
// travel to
bool calculateCostMap(const std::vector<float>& cost_path_candidates,
                      std::vector<int>& cost_idx_sorted) {
  if (cost_path_candidates.empty()) {
    ROS_WARN("\033[1;31mbold Empty candidates vector!\033[0m\n");
    return 1;
  } else {
    cost_idx_sorted.resize(cost_path_candidates.size());
    std::iota(cost_idx_sorted.begin(), cost_idx_sorted.end(), 0);

    std::sort(cost_idx_sorted.begin(), cost_idx_sorted.end(),
              [&cost_path_candidates](size_t i1, size_t i2) {
                return cost_path_candidates[i1] < cost_path_candidates[i2];
              });
    return 0;
  }
}

void printHistogram(Histogram hist, std::vector<int> z_FOV_idx, int e_FOV_min,
                    int e_FOV_max, int e_chosen, int z_chosen,
                    double resolution) {
  int z_dim = 360 / resolution;
  int e_dim = 180 / resolution;

  for (int e_ind = 0; e_ind < e_dim; e_ind++) {
    for (int z_ind = 0; z_ind < z_dim; z_ind++) {
      if (e_chosen == e_ind && z_chosen == z_ind) {
        std::cout << "\033[1;31m" << hist.get_bin(e_ind, z_ind) << " \033[0m";
      } else if (std::find(z_FOV_idx.begin(), z_FOV_idx.end(), z_ind) !=
                     z_FOV_idx.end() &&
                 e_ind > e_FOV_min && e_ind < e_FOV_max) {
        std::cout << "\033[1;32m" << hist.get_bin(e_ind, z_ind) << " \033[0m";
      } else {
        std::cout << hist.get_bin(e_ind, z_ind) << " ";
      }
    }
    std::cout << "\n";
  }
  std::cout << "--------------------------------------\n";
}

bool getDirectionFromTree(
    PolarPoint& p_pol,
    const std::vector<geometry_msgs::Point>& path_node_positions,
    const Eigen::Vector3f& position) {
  int size = path_node_positions.size();
  bool tree_available = true;

  if (size > 0) {
    int min_dist_idx = 0;
    int second_min_dist_idx = 0;
    double min_dist = HUGE_VAL;
    double second_min_dist = HUGE_VAL;
    double node_distance =
        (toEigen(path_node_positions[0]) - toEigen(path_node_positions[1]))
            .norm();

    std::vector<double> distances;
    for (int i = 0; i < size; i++) {
      distances.push_back((position - toEigen(path_node_positions[i])).norm());
      if (distances[i] < min_dist) {
        second_min_dist_idx = min_dist_idx;
        second_min_dist = min_dist;
        min_dist = distances[i];
        min_dist_idx = i;
      } else if (distances[i] < second_min_dist) {
        second_min_dist = distances[i];
        second_min_dist_idx = i;
      }
    }
    int wp_idx = std::min(min_dist_idx, second_min_dist_idx);
    if (min_dist > 3.0 || wp_idx == 0) {
      tree_available = false;
    } else {
      double cos_alpha = (node_distance * node_distance +
                          distances[wp_idx] * distances[wp_idx] -
                          distances[wp_idx + 1] * distances[wp_idx + 1]) /
                         (2 * node_distance * distances[wp_idx]);
      double l_front = distances[wp_idx] * cos_alpha;
      double l_frac = l_front / node_distance;

      Eigen::Vector3f mean_point =
          (1.f - l_frac) * toEigen(path_node_positions[wp_idx - 1]) +
          l_frac * toEigen(path_node_positions[wp_idx]);

      p_pol = cartesianToPolar(mean_point, position);
      p_pol.r = 0.0;
    }
  } else {
    tree_available = false;
  }
  return tree_available;
}
}
