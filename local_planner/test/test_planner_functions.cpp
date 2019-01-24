#include <gtest/gtest.h>
#include <cmath>

#include "../src/nodes/planner_functions.h"

#include "../src/nodes/common.h"

using namespace avoidance;

void check_indexes(nav_msgs::GridCells &test,
                   std::vector<std::pair<int, int>> &truth, int resolution) {
  for (int i = 0, j = 0; i < test.cells.size(), j < truth.size(); i++, j++) {
    std::pair<int, int> cell_idx(
        elevationAngletoIndex(test.cells[i].x, resolution),
        azimuthAngletoIndex(test.cells[i].y, resolution));
    EXPECT_EQ(truth[j], cell_idx);
  }
}

void check_indexes(nav_msgs::GridCells &test, int truth_idx_e, int truth_idx_z,
                   int resolution) {
  for (int i = 0; i < test.cells.size(); i++) {
    EXPECT_EQ(truth_idx_e, elevationAngletoIndex(test.cells[i].x, resolution));
    EXPECT_EQ(truth_idx_z, azimuthAngletoIndex(test.cells[i].y, resolution));
  }
}

TEST(PlannerFunctions, generateNewHistogramEmpty) {
  // GIVEN: an empty pointcloud
  pcl::PointCloud<pcl::PointXYZ> empty_cloud;
  Histogram histogram_output = Histogram(ALPHA_RES);
  geometry_msgs::PoseStamped location;
  location.pose.position.x = 0;
  location.pose.position.y = 0;
  location.pose.position.z = 0;

  // WHEN: we build a histogram
  generateNewHistogram(histogram_output, empty_cloud, location);

  // THEN: the histogram should be all zeros
  for (int e = 0; e < GRID_LENGTH_E; e++) {
    for (int z = 0; z < GRID_LENGTH_Z; z++) {
      EXPECT_DOUBLE_EQ(0.0, histogram_output.get_bin(e, z));
    }
  }
}

TEST(PlannerFunctions, generateNewHistogramSpecificCells) {
  // GIVEN: a pointcloud with an object of one cell size
  Histogram histogram_output = Histogram(ALPHA_RES);
  geometry_msgs::PoseStamped location;
  location.pose.position.x = 0;
  location.pose.position.y = 0;
  location.pose.position.z = 0;
  float distance = 1.0;

  std::vector<float> e_angle_filled = {-90, -30, 0, 20, 40, 90};
  std::vector<float> z_angle_filled = {-180, -50, 0, 59, 100, 175};
  std::vector<Eigen::Vector3f> middle_of_cell;

  for (auto i : e_angle_filled) {
    for (auto j : z_angle_filled) {
      PolarPoint p_pol = {};
      p_pol.e = i;
      p_pol.z = j;
      p_pol.r = distance;
      middle_of_cell.push_back(PolarToCartesian(p_pol, location.pose.position));
    }
  }

  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (int i = 0; i < middle_of_cell.size(); i++) {
    for (int j = 0; j < 1; j++) {
      cloud.push_back(toXYZ(middle_of_cell[i]));
    }
  }

  // WHEN: we build a histogram
  generateNewHistogram(histogram_output, cloud, location);

  // THEN: the filled cells in the histogram should be one and the others be
  // zeros

  std::vector<int> e_index;
  std::vector<int> z_index;
  for (int i = 0; i < e_angle_filled.size(); i++) {
    PolarPoint p_pol = {};
    p_pol.e = e_angle_filled[i];
    p_pol.z = z_angle_filled[i];
    e_index.push_back(PolarToHistogramIndex(p_pol, ALPHA_RES).y());
    z_index.push_back(PolarToHistogramIndex(p_pol, ALPHA_RES).x());
  }

  for (int e = 0; e < GRID_LENGTH_E; e++) {
    for (int z = 0; z < GRID_LENGTH_Z; z++) {
      bool e_found =
          std::find(e_index.begin(), e_index.end(), e) != e_index.end();
      bool z_found =
          std::find(z_index.begin(), z_index.end(), z) != z_index.end();
      if (e_found && z_found) {
        EXPECT_DOUBLE_EQ(1.0, histogram_output.get_bin(e, z)) << z << ", " << e;
      } else {
        EXPECT_DOUBLE_EQ(0.0, histogram_output.get_bin(e, z)) << z << ", " << e;
      }
    }
  }
}

TEST(PlannerFunctions, calculateFOV) {
  // GIVEN: the horizontal and vertical Field of View, the vehicle yaw and pitc
  double h_fov = 90.0;
  double v_fov = 45.0;
  double yaw_z_greater_grid_length =
      3.14;  // z_FOV_max >= GRID_LENGTH_Z && z_FOV_min >= GRID_LENGTH_Z
  double yaw_z_max_greater_grid =
      -2.3;  // z_FOV_max >= GRID_LENGTH_Z && z_FOV_min < GRID_LENGTH_Z
  double yaw_z_min_smaller_zero = 3.9;  // z_FOV_min < 0 && z_FOV_max >= 0
  double yaw_z_smaller_zero = 5.6;      // z_FOV_max < 0 && z_FOV_min < 0
  double pitch = 0.0;

  // WHEN: we calculate the Field of View
  std::vector<int> z_FOV_idx_z_greater_grid_length;
  std::vector<int> z_FOV_idx_z_max_greater_grid;
  std::vector<int> z_FOV_idx3_z_min_smaller_zero;
  std::vector<int> z_FOV_idx_z_smaller_zero;
  int e_FOV_min;
  int e_FOV_max;

  calculateFOV(h_fov, v_fov, z_FOV_idx_z_greater_grid_length, e_FOV_min,
               e_FOV_max, yaw_z_greater_grid_length, pitch);
  calculateFOV(h_fov, v_fov, z_FOV_idx_z_max_greater_grid, e_FOV_min, e_FOV_max,
               yaw_z_max_greater_grid, pitch);
  calculateFOV(h_fov, v_fov, z_FOV_idx3_z_min_smaller_zero, e_FOV_min,
               e_FOV_max, yaw_z_min_smaller_zero, pitch);
  calculateFOV(h_fov, v_fov, z_FOV_idx_z_smaller_zero, e_FOV_min, e_FOV_max,
               yaw_z_smaller_zero, pitch);

  // THEN: we expect polar histogram indexes that are in the Field of View
  std::vector<int> output_z_greater_grid_length = {
      7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21};
  std::vector<int> output_z_max_greater_grid = {0, 1, 2,  3,  4,  5,  6, 7,
                                                8, 9, 10, 11, 12, 58, 59};
  std::vector<int> output_z_min_smaller_zero = {0, 1, 2,  3,  4,  5,  6, 7,
                                                8, 9, 10, 11, 12, 13, 59};
  std::vector<int> output_z_smaller_zero = {43, 44, 45, 46, 47, 48, 49, 50,
                                            51, 52, 53, 54, 55, 56, 57, 58};

  EXPECT_EQ(18, e_FOV_max);
  EXPECT_EQ(10, e_FOV_min);
  for (size_t i = 0; i < z_FOV_idx_z_greater_grid_length.size(); i++) {
    EXPECT_EQ(output_z_greater_grid_length.at(i),
              z_FOV_idx_z_greater_grid_length.at(i));
  }

  for (size_t i = 0; i < z_FOV_idx_z_max_greater_grid.size(); i++) {
    EXPECT_EQ(output_z_max_greater_grid.at(i),
              z_FOV_idx_z_max_greater_grid.at(i));
  }

  for (size_t i = 0; i < z_FOV_idx3_z_min_smaller_zero.size(); i++) {
    EXPECT_EQ(output_z_min_smaller_zero.at(i),
              z_FOV_idx3_z_min_smaller_zero.at(i));
  }

  for (size_t i = 0; i < z_FOV_idx_z_smaller_zero.size(); i++) {
    EXPECT_EQ(output_z_smaller_zero.at(i), z_FOV_idx_z_smaller_zero.at(i));
  }
}

TEST(PlannerFunctionsTests, findAllFreeDirections) {
  // GIVEN: empty histogram
  Histogram empty_histogram = Histogram(ALPHA_RES);
  double safety_radius = 15.0;
  int resolution_alpha = ALPHA_RES;
  // all the variables below aren't used by the test
  const Eigen::Vector3f goal(2.0f, 0.0f, 2.0f);
  const Eigen::Vector3f position(0.0f, 0.0f, 2.0f);
  const Eigen::Vector3f position_old(-1.0f, 0.0f, 2.0f);
  double goal_cost_param = 2.0;
  double smooth_cost_param = 1.5;
  double height_change_cost_param_adapted = 4.0;
  double height_change_cost_param = 4.0;
  bool only_yawed = false;

  // WHEN: we look for free directions
  nav_msgs::GridCells path_candidates, path_selected, path_rejected,
      path_blocked, path_waypoints;
  std::vector<float> cost_path_candidates;  // not needed

  findFreeDirections(empty_histogram, safety_radius, path_candidates,
                     path_selected, path_rejected, path_blocked, path_waypoints,
                     cost_path_candidates, goal, position, position_old,
                     goal_cost_param, smooth_cost_param,
                     height_change_cost_param_adapted, height_change_cost_param,
                     only_yawed, resolution_alpha);

  // THEN: all directions should be classified as path_candidates
  EXPECT_EQ(
      std::floor(360 / resolution_alpha) * std::floor(180 / resolution_alpha),
      path_candidates.cells.size());
  EXPECT_EQ(0, path_rejected.cells.size());
  EXPECT_EQ(0, path_blocked.cells.size());
}

TEST(PlannerFunctionsTests, findFreeDirectionsNoWrap) {
  // GIVEN: an histogram with a rejected cell, no wrapping of blocked cells
  int resolution_alpha = 5;
  double safety_radius = 5.0;
  Histogram histogram = Histogram(resolution_alpha);
  int obstacle_idx_e = 5;
  int obstacle_idx_z = 10;
  histogram.set_bin(obstacle_idx_e, obstacle_idx_z,
                    histogram.get_bin(obstacle_idx_e, obstacle_idx_z) + 1);
  // all the variables below aren't used by the test
  const Eigen::Vector3f goal(2.0f, 0.0f, 2.0f);
  const Eigen::Vector3f position(0.0f, 0.0f, 2.0f);
  const Eigen::Vector3f position_old(-1.0f, 0.0f, 2.0f);
  double goal_cost_param = 2.0;
  double smooth_cost_param = 1.5;
  double height_change_cost_param_adapted = 4.0;
  double height_change_cost_param = 4.0;
  bool only_yawed = false;

  // expected output
  std::vector<std::pair<int, int>> blocked;
  std::vector<std::pair<int, int>> free;
  for (int e = 0; e < (180 / resolution_alpha); e++) {
    for (int z = 0; z < (360 / resolution_alpha); z++) {
      if (!(z <= 11 && z >= 9 && e >= 4 && e <= 6)) {
        free.push_back(std::make_pair(e, z));
      } else {
        if (!(e == obstacle_idx_e && z == obstacle_idx_z)) {
          blocked.push_back(std::make_pair(e, z));
        }
      }
    }
  }

  // WHEN: we look for free directions
  nav_msgs::GridCells path_candidates, path_selected, path_rejected,
      path_blocked, path_waypoints;
  std::vector<float> cost_path_candidates;  // not needed

  findFreeDirections(histogram, safety_radius, path_candidates, path_selected,
                     path_rejected, path_blocked, path_waypoints,
                     cost_path_candidates, goal, position, position_old,
                     goal_cost_param, smooth_cost_param,
                     height_change_cost_param_adapted, height_change_cost_param,
                     only_yawed, resolution_alpha);

  // THEN: we should have one rejected cell and the 8 neightbooring cells
  // blocked
  check_indexes(path_blocked, blocked, resolution_alpha);
  check_indexes(path_candidates, free, resolution_alpha);
  check_indexes(path_rejected, obstacle_idx_e, obstacle_idx_z,
                resolution_alpha);

  EXPECT_EQ(1, path_rejected.cells.size());
  EXPECT_EQ(blocked.size(), path_blocked.cells.size());
  EXPECT_EQ(
      std::floor(360 / resolution_alpha) * std::floor(180 / resolution_alpha) -
          1 - blocked.size(),
      path_candidates.cells.size());
}

TEST(PlannerFunctionsTests, findFreeDirectionsWrapLeft) {
  // GIVEN: a histogram with an obstacle at the first azimuth index
  int resolution_alpha = 5;
  double safety_radius = 5.0;
  Histogram histogram = Histogram(resolution_alpha);
  int obstacle_idx_e = 5;
  int obstacle_idx_z = 0;
  histogram.set_bin(obstacle_idx_e, obstacle_idx_z,
                    histogram.get_bin(obstacle_idx_e, obstacle_idx_z) + 1);

  // all the variables below aren't used by the test
  const Eigen::Vector3f goal(2.0f, 0.0f, 2.0f);
  const Eigen::Vector3f position(0.0f, 0.0f, 2.0f);
  const Eigen::Vector3f position_old(-1.0f, 0.0f, 2.0f);
  double goal_cost_param = 2.0;
  double smooth_cost_param = 1.5;
  double height_change_cost_param_adapted = 4.0;
  double height_change_cost_param = 4.0;
  bool only_yawed = false;

  // expected output
  std::vector<std::pair<int, int>> blocked;
  std::vector<std::pair<int, int>> free;
  for (int e = 0; e < (180 / resolution_alpha); e++) {
    for (int z = 0; z < (360 / resolution_alpha); z++) {
      if (!((z == 1 || z == 0 || z == 71) && e >= 4 && e <= 6)) {
        free.push_back(std::make_pair(e, z));
      } else {
        if (!(e == obstacle_idx_e && z == obstacle_idx_z)) {
          blocked.push_back(std::make_pair(e, z));
        }
      }
    }
  }

  // WHEN: we look for free directions
  nav_msgs::GridCells path_candidates, path_selected, path_rejected,
      path_blocked, path_waypoints;
  std::vector<float> cost_path_candidates;  // not needed

  findFreeDirections(histogram, safety_radius, path_candidates, path_selected,
                     path_rejected, path_blocked, path_waypoints,
                     cost_path_candidates, goal, position, position_old,
                     goal_cost_param, smooth_cost_param,
                     height_change_cost_param_adapted, height_change_cost_param,
                     only_yawed, resolution_alpha);

  // THEN: we should get one rejected cell, the five neighboring cells blocked
  // plus three other blocked cells at the same elevation index and the last
  // azimuth index
  check_indexes(path_blocked, blocked, resolution_alpha);
  check_indexes(path_candidates, free, resolution_alpha);
  check_indexes(path_rejected, obstacle_idx_e, obstacle_idx_z,
                resolution_alpha);

  EXPECT_EQ(1, path_rejected.cells.size());
  EXPECT_EQ(blocked.size(), path_blocked.cells.size());
  EXPECT_EQ(
      std::floor(360 / resolution_alpha) * std::floor(180 / resolution_alpha) -
          1 - blocked.size(),
      path_candidates.cells.size());
}

TEST(PlannerFunctionsTests, findFreeDirectionsWrapRight) {
  // GIVEN: a histogram with a obstacle at the last azimuth index
  int resolution_alpha = 5;
  double safety_radius = 5.0;
  Histogram histogram = Histogram(resolution_alpha);
  int obstacle_idx_e = 5;
  int obstacle_idx_z = 71;
  histogram.set_bin(obstacle_idx_e, obstacle_idx_z,
                    histogram.get_bin(obstacle_idx_e, obstacle_idx_z) + 1);
  // all the variables below aren't used by the test
  const Eigen::Vector3f goal(2.0f, 0.0f, 2.0f);
  const Eigen::Vector3f position(0.0f, 0.0f, 2.0f);
  const Eigen::Vector3f position_old(-1.0f, 0.0f, 2.0f);
  double goal_cost_param = 2.0;
  double smooth_cost_param = 1.5;
  double height_change_cost_param_adapted = 4.0;
  double height_change_cost_param = 4.0;
  bool only_yawed = false;

  // expected output
  std::vector<std::pair<int, int>> blocked;
  std::vector<std::pair<int, int>> free;
  for (int e = 0; e < (180 / resolution_alpha); e++) {
    for (int z = 0; z < (360 / resolution_alpha); z++) {
      if (!((z == 70 || z == 71 || z == 0) && e >= 4 && e <= 6)) {
        free.push_back(std::make_pair(e, z));
      } else {
        if (!(e == obstacle_idx_e && z == obstacle_idx_z)) {
          blocked.push_back(std::make_pair(e, z));
        }
      }
    }
  }

  // WHEN: we look for free directions
  nav_msgs::GridCells path_candidates, path_selected, path_rejected,
      path_blocked, path_waypoints;
  std::vector<float> cost_path_candidates;  // not needed

  findFreeDirections(histogram, safety_radius, path_candidates, path_selected,
                     path_rejected, path_blocked, path_waypoints,
                     cost_path_candidates, goal, position, position_old,
                     goal_cost_param, smooth_cost_param,
                     height_change_cost_param_adapted, height_change_cost_param,
                     only_yawed, resolution_alpha);

  // THEN: we should get one rejected cell, the five neighboring cells blocked
  // plus three other blocked cells at the same elevation index and the first
  // azimuth index
  check_indexes(path_blocked, blocked, resolution_alpha);
  check_indexes(path_candidates, free, resolution_alpha);
  check_indexes(path_rejected, obstacle_idx_e, obstacle_idx_z,
                resolution_alpha);

  EXPECT_EQ(1, path_rejected.cells.size());
  EXPECT_EQ(blocked.size(), path_blocked.cells.size());
  EXPECT_EQ(
      std::floor(360 / resolution_alpha) * std::floor(180 / resolution_alpha) -
          1 - blocked.size(),
      path_candidates.cells.size());
}

TEST(PlannerFunctionsTests, findFreeDirectionsWrapUp) {
  // GIVEN:a histogram with a obstacle at the first elevation index
  int resolution_alpha = 8;
  double safety_radius = 16.0;
  Histogram histogram = Histogram(resolution_alpha);
  int obstacle_idx_e = 0;
  int obstacle_idx_z = 19;
  histogram.set_bin(obstacle_idx_e, obstacle_idx_z,
                    histogram.get_bin(obstacle_idx_e, obstacle_idx_z) + 1);
  // all the variables below aren't used by the test
  const Eigen::Vector3f goal(2.0f, 0.0f, 2.0f);
  const Eigen::Vector3f position(0.0f, 0.0f, 2.0f);
  const Eigen::Vector3f position_old(-1.0f, 0.0f, 2.0f);
  double goal_cost_param = 2.0;
  double smooth_cost_param = 1.5;
  double height_change_cost_param_adapted = 4.0;
  double height_change_cost_param = 4.0;
  bool only_yawed = false;

  // expected output
  std::vector<std::pair<int, int>> blocked;
  std::vector<std::pair<int, int>> free;

  for (int e = 0; e < (180 / resolution_alpha); e++) {
    for (int z = 0; z < (360 / resolution_alpha); z++) {
      if (!(e >= 0 && e <= 2 && z >= 17 && z <= 21) &&
          !(e >= 0 && e <= 1 && z >= 40 && z <= 44)) {
        free.push_back(std::make_pair(e, z));
      } else {
        if (!(e == obstacle_idx_e && z == obstacle_idx_z)) {
          blocked.push_back(std::make_pair(e, z));
        }
      }
    }
  }

  // WHEN: we look for free directions
  nav_msgs::GridCells path_candidates, path_selected, path_rejected,
      path_blocked, path_waypoints;
  std::vector<float> cost_path_candidates;  // not needed

  findFreeDirections(histogram, safety_radius, path_candidates, path_selected,
                     path_rejected, path_blocked, path_waypoints,
                     cost_path_candidates, goal, position, position_old,
                     goal_cost_param, smooth_cost_param,
                     height_change_cost_param_adapted, height_change_cost_param,
                     only_yawed, resolution_alpha);

  // THEN: we should get one rejected cell, the 14 neighboring cells blocked
  // plus 10 other blocked cells at the same elevation index and azimuth index
  // shifted by 180 degrees
  check_indexes(path_blocked, blocked, resolution_alpha);
  check_indexes(path_candidates, free, resolution_alpha);
  check_indexes(path_rejected, obstacle_idx_e, obstacle_idx_z,
                resolution_alpha);

  EXPECT_EQ(1, path_rejected.cells.size());
  EXPECT_EQ(blocked.size(), path_blocked.cells.size());
  EXPECT_EQ(
      std::floor(360 / resolution_alpha) * std::floor(180 / resolution_alpha) -
          1 - blocked.size(),
      path_candidates.cells.size());
}

TEST(PlannerFunctionsTests, findFreeDirectionsWrapDown) {
  // GIVEN: a histogram with a obstacle at the last elevation index
  int resolution_alpha = 10;
  double safety_radius = 10.0;
  Histogram histogram = Histogram(resolution_alpha);
  int obstacle_idx_e = 17;
  int obstacle_idx_z = 4;
  histogram.set_bin(obstacle_idx_e, obstacle_idx_z,
                    histogram.get_bin(obstacle_idx_e, obstacle_idx_z) + 1);
  // all the variables below aren't used by the test
  const Eigen::Vector3f goal(2.0f, 0.0f, 2.0f);
  const Eigen::Vector3f position(0.0f, 0.0f, 2.0f);
  const Eigen::Vector3f position_old(-1.0f, 0.0f, 2.0f);
  double goal_cost_param = 2.0;
  double smooth_cost_param = 1.5;
  double height_change_cost_param_adapted = 4.0;
  double height_change_cost_param = 4.0;
  bool only_yawed = false;

  // expected output
  std::vector<std::pair<int, int>> blocked;
  std::vector<std::pair<int, int>> free;
  for (int e = 0; e < (180 / resolution_alpha); e++) {
    for (int z = 0; z < (360 / resolution_alpha); z++) {
      if (!((e == 16 || e == 17) && (z >= 3 && z <= 5)) &&
          !(e == 17 && (z >= 21 && z <= 23))) {
        free.push_back(std::make_pair(e, z));
      } else {
        if (!(e == obstacle_idx_e && z == obstacle_idx_z)) {
          blocked.push_back(std::make_pair(e, z));
        }
      }
    }
  }

  // WHEN: we look for free directions
  nav_msgs::GridCells path_candidates, path_selected, path_rejected,
      path_blocked, path_waypoints;
  std::vector<float> cost_path_candidates;  // not needed

  findFreeDirections(histogram, safety_radius, path_candidates, path_selected,
                     path_rejected, path_blocked, path_waypoints,
                     cost_path_candidates, goal, position, position_old,
                     goal_cost_param, smooth_cost_param,
                     height_change_cost_param_adapted, height_change_cost_param,
                     only_yawed, resolution_alpha);

  // THEN: we should get one rejected cell, the 5 neighboring cells blocked
  // plus 3 other blocked cells at the same elevation index and azimuth index
  // shifted by 180 degrees
  check_indexes(path_blocked, blocked, resolution_alpha);
  check_indexes(path_candidates, free, resolution_alpha);
  check_indexes(path_rejected, obstacle_idx_e, obstacle_idx_z,
                resolution_alpha);

  EXPECT_EQ(1, path_rejected.cells.size());
  EXPECT_EQ(blocked.size(), path_blocked.cells.size());
  EXPECT_EQ(
      std::floor(360 / resolution_alpha) * std::floor(180 / resolution_alpha) -
          1 - blocked.size(),
      path_candidates.cells.size());
}

TEST(PlannerFunctionsTests, filterPointCloud) {
  // GIVEN: two point clouds
  const Eigen::Vector3f position(1.5f, 1.0f, 4.5f);
  Eigen::Vector3f closest(0.7f, 0.3f, -0.5f);
  pcl::PointCloud<pcl::PointXYZ> p1;
  p1.push_back(toXYZ(position + Eigen::Vector3f(1.1f, 0.8f, 0.1f)));
  p1.push_back(toXYZ(position + Eigen::Vector3f(2.2f, 1.0f, 1.0f)));
  p1.push_back(toXYZ(position + Eigen::Vector3f(1.0f, -3.0f, 1.0f)));
  p1.push_back(toXYZ(
      position + Eigen::Vector3f(0.7f, 0.3f, -0.5f)));  // < min_dist_backoff
  p1.push_back(toXYZ(position + Eigen::Vector3f(-1.0f, 1.0f, 1.0f)));
  p1.push_back(toXYZ(position + Eigen::Vector3f(-1.0f, -1.1f, 3.5f)));

  pcl::PointCloud<pcl::PointXYZ> p2;
  p2.push_back(toXYZ(
      position + Eigen::Vector3f(1.0f, 5.0f, 1.0f)));  // > histogram_box.radius
  p2.push_back(
      toXYZ(position +
            Eigen::Vector3f(100.0f, 5.0f, 1.0f)));  // > histogram_box.radius
  p2.push_back(toXYZ(
      position + Eigen::Vector3f(0.1f, 0.05f, 0.05f)));  // < min_realsense_dist

  std::vector<pcl::PointCloud<pcl::PointXYZ>> complete_cloud;
  complete_cloud.push_back(p1);
  complete_cloud.push_back(p2);
  double min_dist_backoff = 1.0;
  Box histogram_box(5.0);
  histogram_box.setBoxLimits(toPoint(position), 4.5);
  double min_realsense_dist = 0.2;

  pcl::PointCloud<pcl::PointXYZ> cropped_cloud, cropped_cloud2;
  Eigen::Vector3f closest_point, closest_point2;
  double distance_to_closest_point, distance_to_closest_point2;
  int counter_backoff, counter_backoff2;

  // WHEN: we filter the PointCloud with different values of min_cloud_size
  filterPointCloud(cropped_cloud, closest_point, distance_to_closest_point,
                   counter_backoff, complete_cloud, 5.0, min_dist_backoff,
                   histogram_box, position, min_realsense_dist);

  filterPointCloud(cropped_cloud2, closest_point2, distance_to_closest_point2,
                   counter_backoff2, complete_cloud, 20.0, min_dist_backoff,
                   histogram_box, position, min_realsense_dist);

  // THEN: we expect cropped_cloud to have 6 points and a backoff point, while
  // cropped_cloud2 to be empty
  EXPECT_FLOAT_EQ((position + closest).x(), closest_point.x());
  EXPECT_FLOAT_EQ((position + closest).y(), closest_point.y());
  EXPECT_FLOAT_EQ((position + closest).z(), closest_point.z());
  EXPECT_FLOAT_EQ(closest.norm(), distance_to_closest_point);
  EXPECT_EQ(1, counter_backoff);
  for (int i = 0; i < p1.points.size(); i++) {
    bool same_point = (p1.points[i].x == cropped_cloud.points[i].x) &&
                      (p1.points[i].y == cropped_cloud.points[i].y) &&
                      (p1.points[i].z == cropped_cloud.points[i].z);
    ASSERT_TRUE(same_point);
  }

  EXPECT_EQ(0, cropped_cloud2.points.size());
}
