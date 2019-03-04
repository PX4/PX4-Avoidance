#include "local_planner/planner_functions.h"

#include "local_planner/common.h"

#include <ros/console.h>

#include <numeric>

namespace avoidance {

// trim the point cloud so that only points inside the bounding box are
// considered
void filterPointCloud(
    pcl::PointCloud<pcl::PointXYZ>& cropped_cloud,
    Eigen::Vector3f& closest_point, float& distance_to_closest_point,
    int& counter_backoff,
    const std::vector<pcl::PointCloud<pcl::PointXYZ>>& complete_cloud,
    int min_cloud_size, float min_dist_backoff, Box histogram_box,
    const Eigen::Vector3f& position, float min_realsense_dist) {
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
void calculateFOV(float h_fov, float v_fov, std::vector<int>& z_FOV_idx,
                  int& e_FOV_min, int& e_FOV_max, float yaw, float pitch) {
  int z_FOV_max =
      static_cast<int>(std::round((-yaw * RAD_TO_DEG + h_fov / 2.0f + 270.0f) /
                                  static_cast<float>(ALPHA_RES))) -
      1;
  int z_FOV_min =
      static_cast<int>(std::round((-yaw * RAD_TO_DEG - h_fov / 2.0f + 270.0f) /
                                  static_cast<float>(ALPHA_RES))) -
      1;
  e_FOV_max =
      static_cast<int>(std::round((-pitch * RAD_TO_DEG + v_fov / 2.0f + 90.0f) /
                                  static_cast<float>(ALPHA_RES))) -
      1;
  e_FOV_min =
      static_cast<int>(std::round((-pitch * RAD_TO_DEG - v_fov / 2.0f + 90.0f) /
                                  static_cast<float>(ALPHA_RES))) -
      1;

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
    const std::vector<int>& reprojected_points_age,
    const Eigen::Vector3f& position) {
  Eigen::MatrixXi counter(GRID_LENGTH_E / 2, GRID_LENGTH_Z / 2);
  counter.fill(0);

  for (size_t i = 0; i < reprojected_points.points.size(); i++) {
    PolarPoint p_pol =
        cartesianToPolar(toEigen(reprojected_points.points[i]), position);
    Eigen::Vector2i p_ind = polarToHistogramIndex(p_pol, 2 * ALPHA_RES);
    float point_distance =
        (position - toEigen(reprojected_points.points[i])).norm();

    counter(p_ind.y(), p_ind.x()) += 1;
    polar_histogram_est.set_age(
        p_ind.y(), p_ind.x(),
        polar_histogram_est.get_age(p_ind.y(), p_ind.x()) +
            reprojected_points_age[i]);
    polar_histogram_est.set_dist(
        p_ind.y(), p_ind.x(),
        polar_histogram_est.get_dist(p_ind.y(), p_ind.x()) + point_distance);
  }

  for (int e = 0; e < GRID_LENGTH_E / 2; e++) {
    for (int z = 0; z < GRID_LENGTH_Z / 2; z++) {
      if (counter(e, z) >= 6) {
        polar_histogram_est.set_dist(
            e, z, polar_histogram_est.get_dist(e, z) / counter(e, z));
        polar_histogram_est.set_age(
            e, z, static_cast<int>(polar_histogram_est.get_age(e, z) /
                                   counter(e, z)));
      } else {  // not enough points to confidently block cell
        polar_histogram_est.set_dist(e, z, 0.f);
        polar_histogram_est.set_age(e, z, 0);
      }
    }
  }

  // Upsample propagated histogram
  polar_histogram_est.upsample();
}

// Generate new histogram from pointcloud
void generateNewHistogram(Histogram& polar_histogram,
                          const pcl::PointCloud<pcl::PointXYZ>& cropped_cloud,
                          const Eigen::Vector3f& position) {
  Eigen::MatrixXi counter(GRID_LENGTH_E, GRID_LENGTH_Z);
  counter.fill(0);
  for (auto xyz : cropped_cloud) {
    Eigen::Vector3f p = toEigen(xyz);
    float dist = (p - position).norm();
    PolarPoint p_pol = cartesianToPolar(p, position);
    Eigen::Vector2i p_ind = polarToHistogramIndex(p_pol, ALPHA_RES);

    counter(p_ind.y(), p_ind.x()) += 1;
    polar_histogram.set_dist(
        p_ind.y(), p_ind.x(),
        polar_histogram.get_dist(p_ind.y(), p_ind.x()) + dist);
  }

  // Normalize and get mean in distance bins
  for (int e = 0; e < GRID_LENGTH_E; e++) {
    for (int z = 0; z < GRID_LENGTH_Z; z++) {
      if (counter(e, z) > 0) {
        polar_histogram.set_dist(
            e, z, polar_histogram.get_dist(e, z) / counter(e, z));
      } else {
        polar_histogram.set_dist(e, z, 0.f);
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
  for (int z = 0; z < GRID_LENGTH_Z; z++) {
    bool inside_FOV_z =
        std::find(z_FOV_idx.begin(), z_FOV_idx.end(), z) != z_FOV_idx.end();
    for (int e = 0; e < GRID_LENGTH_E; e++) {
      if (inside_FOV_z && e > e_FOV_min && e < e_FOV_max) {  // inside FOV
        if (new_hist.get_dist(e, z) > 0) {
          new_hist.set_age(e, z, 1);
          hist_empty = false;
        }
      } else {
        if (propagated_hist.get_dist(e, z) > 0) {
          if (waypoint_outside_FOV) {
            new_hist.set_age(e, z, propagated_hist.get_age(e, z));
          } else {
            new_hist.set_age(e, z, propagated_hist.get_age(e, z) + 1);
          }
          hist_empty = false;
        }
        if (new_hist.get_dist(e, z) > 0) {
          new_hist.set_age(e, z, 1);
          hist_empty = false;
        }
        if (propagated_hist.get_dist(e, z) > 0 &&
            new_hist.get_dist(e, z) < FLT_MIN) {
          new_hist.set_dist(e, z, propagated_hist.get_dist(e, z));
        }
      }
    }
  }
}

void compressHistogramElevation(Histogram& new_hist,
                                const Histogram& input_hist) {
  float vertical_FOV_range_sensor = 20.0;
  PolarPoint p_pol_lower(-1.0f * vertical_FOV_range_sensor / 2.0f, 0.0f, 0.0f);
  PolarPoint p_pol_upper(vertical_FOV_range_sensor / 2.0f, 0.0f, 0.0f);
  Eigen::Vector2i p_ind_lower = polarToHistogramIndex(p_pol_lower, ALPHA_RES);
  Eigen::Vector2i p_ind_upper = polarToHistogramIndex(p_pol_upper, ALPHA_RES);

  for (int e = p_ind_lower.y(); e <= p_ind_upper.y(); e++) {
    for (int z = 0; z < GRID_LENGTH_Z; z++) {
      if (input_hist.get_dist(e, z) > 0) {
        if (input_hist.get_dist(e, z) < new_hist.get_dist(0, z) ||
            (new_hist.get_dist(0, z) == 0.f))
          new_hist.set_dist(0, z, input_hist.get_dist(e, z));
      }
    }
  }
}

void getCostMatrix(const Histogram& histogram, const Eigen::Vector3f& goal,
                   const Eigen::Vector3f& position, const float heading,
                   const Eigen::Vector3f& last_sent_waypoint,
                   costParameters cost_params, bool only_yawed,
                   Eigen::MatrixXf& cost_matrix) {
  Eigen::MatrixXf distance_matrix(GRID_LENGTH_E, GRID_LENGTH_Z);
  distance_matrix.fill(NAN);
  float distance_cost = 0.f;
  float other_costs = 0.f;
  // reset cost matrix to zero
  cost_matrix.resize(GRID_LENGTH_E, GRID_LENGTH_Z);
  cost_matrix.fill(NAN);

  // fill in cost matrix
  for (int e_index = 0; e_index < GRID_LENGTH_E; e_index++) {
    int z_scale = static_cast<int>(
        std::round(
        1 / std::cos(histogramIndexToPolar(e_index, 0, ALPHA_RES, 1).e *
                     DEG_TO_RAD)));

    z_scale = 1;
    for (int z_index = 0; z_index < GRID_LENGTH_Z; z_index += z_scale) {
      float obstacle_distance = histogram.get_dist(e_index, z_index);
      PolarPoint p_pol =
          histogramIndexToPolar(e_index, z_index, ALPHA_RES, obstacle_distance);

      costFunction(p_pol.e, p_pol.z, obstacle_distance, goal, position, heading,
                   last_sent_waypoint, cost_params, distance_cost, other_costs);
      cost_matrix(e_index, z_index) = other_costs;
      distance_matrix(e_index, z_index) = distance_cost;
    }
    if (z_scale > 1) {
      // horizontally interpolate all of the un-calculated values
      int last_index = 0;
      for (int z_index = z_scale; z_index < GRID_LENGTH_Z; z_index += z_scale) {
        float other_costs_gradient =
            (cost_matrix(e_index, z_index) - cost_matrix(e_index, last_index)) /
            z_scale;
        float distance_cost_gradient = (distance_matrix(e_index, z_index) -
                                        distance_matrix(e_index, last_index)) /
                                       z_scale;
        for (int i = 1; i < z_scale; i++) {
          cost_matrix(e_index, last_index + i) =
              cost_matrix(e_index, last_index) + other_costs_gradient * i;
          distance_matrix(e_index, last_index + i) =
              distance_matrix(e_index, last_index) + distance_cost_gradient * i;
        }
        last_index = z_index;
      }
      
      // special case the last columns wrapping around back to 0
      int clamped_z_scale = GRID_LENGTH_Z - last_index;
      float other_costs_gradient =
          (cost_matrix(e_index, 0) - cost_matrix(e_index, last_index)) /
          clamped_z_scale;
      float distance_cost_gradient =
          (distance_matrix(e_index, 0) - distance_matrix(e_index, last_index)) /
          clamped_z_scale;

      for (int i = 1; i < clamped_z_scale; i++) {
        cost_matrix(e_index, last_index + i) =
            cost_matrix(e_index, last_index) + other_costs_gradient * i;
        distance_matrix(e_index, last_index + i) =
            distance_matrix(e_index, last_index) + distance_cost_gradient * i;
      }
    }
  }

  unsigned int smooth_radius = 1;
  float smoothing_margin_degrees = 45;
  int smoothing_steps = ceil(smoothing_margin_degrees / ALPHA_RES);
  for (int i = 0; i < smoothing_steps; i++) {
    smoothPolarMatrix(distance_matrix, smooth_radius);
  }

  cost_matrix = cost_matrix + distance_matrix;
}

void getBestCandidatesFromCostMatrix(
    const Eigen::MatrixXf& matrix, unsigned int number_of_candidates,
    std::vector<candidateDirection>& candidate_vector) {
  std::priority_queue<candidateDirection, std::vector<candidateDirection>,
                      std::less<candidateDirection>>
      queue;

  for (int row_index = 0; row_index < matrix.rows(); row_index++) {
    for (int col_index = 0; col_index < matrix.cols(); col_index++) {
      PolarPoint p_pol =
          histogramIndexToPolar(row_index, col_index, ALPHA_RES, 1.0);
      float cost = matrix(row_index, col_index);
      candidateDirection candidate(cost, p_pol.e, p_pol.z);

      if (queue.size() < number_of_candidates) {
        queue.push(candidate);
      } else if (candidate < queue.top()) {
        queue.push(candidate);
        queue.pop();
      }
    }
  }
  // copy queue to vector and change order such that lowest cost is at the front
  candidate_vector.clear();
  candidate_vector.reserve(queue.size());
  while (!queue.empty()) {
    candidate_vector.push_back(queue.top());
    queue.pop();
  }
  std::reverse(candidate_vector.begin(), candidate_vector.end());
}

void smoothPolarMatrix(Eigen::MatrixXf& matrix, unsigned int smoothing_radius) {
  // pad matrix by smoothing radius respecting all wrapping rules
  Eigen::MatrixXf matrix_padded;
  padPolarMatrix(matrix, smoothing_radius, matrix_padded);

  // filter matrix (max-mean)
  for (int row_index = smoothing_radius;
       row_index < matrix_padded.rows() - smoothing_radius; row_index++) {
    for (int col_index = smoothing_radius;
         col_index < matrix_padded.cols() - smoothing_radius; col_index++) {
      float original_val = matrix_padded(row_index, col_index);
      float mean_val =
          matrix_padded
              .block(row_index - smoothing_radius, col_index - smoothing_radius,
                     2 * smoothing_radius + 1, 2 * smoothing_radius + 1)
              .mean();
      matrix(row_index - smoothing_radius, col_index - smoothing_radius) =
          std::max(original_val, mean_val);
    }
  }
}

void padPolarMatrix(const Eigen::MatrixXf& matrix, unsigned int n_lines_padding,
                    Eigen::MatrixXf& matrix_padded) {
  matrix_padded.resize(matrix.rows() + 2 * n_lines_padding,
                       matrix.cols() + 2 * n_lines_padding);

  matrix_padded.fill(0.0);
  // middle part
  matrix_padded.block(n_lines_padding, n_lines_padding, matrix.rows(),
                      matrix.cols()) = matrix;

  if (matrix.cols() % 2 > 0) {
    ROS_ERROR("invalid resolution: 180 mod (2* resolution) must be zero");
  }
  int middle_index = floor(matrix.cols() / 2);

  // top border
  matrix_padded.block(0, n_lines_padding, n_lines_padding, middle_index) =
      matrix.block(0, middle_index, n_lines_padding, middle_index)
          .colwise()
          .reverse();
  matrix_padded.block(0, n_lines_padding + middle_index, n_lines_padding,
                      middle_index) =
      matrix.block(0, 0, n_lines_padding, middle_index).colwise().reverse();

  // bottom border
  matrix_padded.block(matrix.rows() + n_lines_padding, n_lines_padding,
                      n_lines_padding, middle_index) =
      matrix
          .block(matrix.rows() - n_lines_padding, middle_index, n_lines_padding,
                 middle_index)
          .colwise()
          .reverse();
  matrix_padded.block(matrix.rows() + n_lines_padding,
                      n_lines_padding + middle_index, n_lines_padding,
                      middle_index) =
      matrix
          .block(matrix.rows() - n_lines_padding, 0, n_lines_padding,
                 middle_index)
          .colwise()
          .reverse();

  // left border
  matrix_padded.block(0, 0, matrix_padded.rows(), n_lines_padding) =
      matrix_padded.block(0, matrix_padded.cols() - 2 * n_lines_padding,
                          matrix_padded.rows(), n_lines_padding);
  // right border
  matrix_padded.block(0, n_lines_padding + matrix.cols(), matrix_padded.rows(),
                      n_lines_padding) =
      matrix_padded.block(0, n_lines_padding, matrix_padded.rows(),
                          n_lines_padding);
}

// costfunction for every free histogram cell
void costFunction(float e_angle, float z_angle, float obstacle_distance,
                  const Eigen::Vector3f& goal, const Eigen::Vector3f& position,
                  const float heading,
                  const Eigen::Vector3f& last_sent_waypoint,
                  costParameters cost_params, float& distance_cost,
                  float& other_costs) {
  float goal_dist = (position - goal).norm();
  PolarPoint p_pol(e_angle, z_angle, goal_dist);
  Eigen::Vector3f projected_candidate =
      polarToCartesian(p_pol, toPoint(position));
  PolarPoint heading_pol(e_angle, heading, goal_dist);
  Eigen::Vector3f projected_heading =
      polarToCartesian(heading_pol, toPoint(position));
  Eigen::Vector3f projected_goal = goal;
  PolarPoint last_wp_pol = cartesianToPolar(last_sent_waypoint, position);
  last_wp_pol.r = goal_dist;
  Eigen::Vector3f projected_last_wp =
      polarToCartesian(last_wp_pol, toPoint(position));

  // goal costs
  float yaw_cost =
      cost_params.goal_cost_param *
      (projected_goal.topRows<2>() - projected_candidate.topRows<2>()).norm();
  float pitch_cost_up = 0.0f;
  float pitch_cost_down = 0.0f;
  if (projected_candidate.z() > projected_goal.z()) {
    pitch_cost_up = cost_params.goal_cost_param *
                    std::abs(projected_goal.z() - projected_candidate.z());
  } else {
    pitch_cost_down = cost_params.goal_cost_param *
                      std::abs(projected_goal.z() - projected_candidate.z());
  }

  // smooth costs
  float yaw_cost_smooth =
      cost_params.smooth_cost_param *
      (projected_last_wp.topRows<2>() - projected_candidate.topRows<2>())
          .norm();
  float pitch_cost_smooth =
      cost_params.smooth_cost_param *
      std::abs(projected_last_wp.z() - projected_candidate.z());

  // heading cost
  float heading_cost =
      cost_params.heading_cost_param *
      (projected_heading.topRows<2>() - projected_candidate.topRows<2>())
          .norm();

  // distance cost
  distance_cost = 0.0f;
  if (obstacle_distance > 0.0f) {
    distance_cost = 12000.0f / obstacle_distance;
  }

  // combine costs
  other_costs = 0.0f;
  other_costs =
      yaw_cost + cost_params.height_change_cost_param_adapted * pitch_cost_up +
      cost_params.height_change_cost_param * pitch_cost_down + yaw_cost_smooth +
      pitch_cost_smooth + heading_cost + distance_cost;
}

bool getDirectionFromTree(
    PolarPoint& p_pol,
    const std::vector<geometry_msgs::Point>& path_node_positions,
    const Eigen::Vector3f& position, const Eigen::Vector3f& goal) {
  int size = path_node_positions.size();
  bool tree_available = true;

  if (size >
      1) {  // path contains at least 2 points (current position and one wp)

    // extend path with a node at the end in goal direction (for smoother
    // transition to direct flight)
    float node_distance =
        (toEigen(path_node_positions[0]) - toEigen(path_node_positions[1]))
            .norm();
    Eigen::Vector3f dir_last_node_to_goal =
        (goal - toEigen(path_node_positions[0])).normalized();
    geometry_msgs::Point goal_node =
        toPoint(toEigen(path_node_positions[0]) +
                node_distance * dir_last_node_to_goal);
    std::vector<geometry_msgs::Point> path_node_positions_extended;
    path_node_positions_extended.push_back(goal_node);
    path_node_positions_extended.insert(path_node_positions_extended.end(),
                                        path_node_positions.begin(),
                                        path_node_positions.end());
    int size_extended = path_node_positions_extended.size();

    // find path nodes between which the drone is currently located:
    // a vector with the distances of each node to the drone is generated
    // the indices of the nodes with smallest and next smallest distance are
    // found
    int min_dist_idx = 0;
    int second_min_dist_idx = 0;
    float min_dist = HUGE_VAL;
    float second_min_dist = HUGE_VAL;
    std::vector<float> distances;
    distances.reserve(size_extended);

    for (int i = 0; i < size_extended; i++) {
      distances.push_back(
          (position - toEigen(path_node_positions_extended[i])).norm());
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

    // the drone is located between the two nodes(min_dist_idx,
    // second_min_dist_idx),
    // wp_idx describes the node further ahead in the path
    int wp_idx = std::min(min_dist_idx, second_min_dist_idx);

    // if drone is too far away from tree or already at last node -> don't use
    // tree
    if (min_dist > 3.0f || wp_idx == 0) {
      tree_available = false;
    } else {
      float cos_alpha = (node_distance * node_distance +
                         distances[wp_idx] * distances[wp_idx] -
                         distances[wp_idx + 1] * distances[wp_idx + 1]) /
                        (2.0f * node_distance * distances[wp_idx]);
      float l_front = distances[wp_idx] * cos_alpha;
      float l_frac = l_front / node_distance;

      Eigen::Vector3f mean_point =
          (1.f - l_frac) * toEigen(path_node_positions_extended[wp_idx - 1]) +
          l_frac * toEigen(path_node_positions_extended[wp_idx]);

      p_pol = cartesianToPolar(mean_point, position);
      p_pol.r = 0.0f;
    }
  } else {
    tree_available = false;
  }
  return tree_available;
}

void printHistogram(Histogram& histogram) {
  std::cout << "------------------------------------------Histogram------------"
               "------------------------------------\n";
  for (int e = 0; e < GRID_LENGTH_E; e++) {
    for (int z = 0; z < GRID_LENGTH_Z; z++) {
      int val = floor(histogram.get_dist(e, z));
      if (val > 99) {
        std::cout << val << " ";
      } else if (val > 9) {
        std::cout << val << "  ";
      } else {
        std::cout << val << "   ";
      }
    }
    std::cout << "\n";
  }
  std::cout << "_______________________________________________________________"
               "____________________________________\n";
}
}
