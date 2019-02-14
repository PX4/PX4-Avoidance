#include <gtest/gtest.h>
#include <cmath>

#include "../include/local_planner/planner_functions.h"

#include "../include/local_planner/common.h"

using namespace avoidance;

void check_indexes(nav_msgs::GridCells &test,
                   std::vector<Eigen::Vector2i> &truth, int resolution) {
  for (int i = 0, j = 0; i < test.cells.size(), j < truth.size(); i++, j++) {
    PolarPoint p_pol(test.cells[i].x, test.cells[i].y, 0.0f);
    Eigen::Vector2i cell_idx = polarToHistogramIndex(p_pol, resolution);

    EXPECT_EQ(truth[j], cell_idx);
  }
}

void check_indexes(nav_msgs::GridCells &test, int truth_idx_e, int truth_idx_z,
                   int resolution) {
  for (int i = 0; i < test.cells.size(); i++) {
    PolarPoint p_pol(test.cells[i].x, test.cells[i].y, 0.0f);
    Eigen::Vector2i cell_idx = polarToHistogramIndex(p_pol, resolution);
    EXPECT_EQ(truth_idx_e, cell_idx.y());
    EXPECT_EQ(truth_idx_z, cell_idx.x());
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
  generateNewHistogram(histogram_output, empty_cloud,
                       toEigen(location.pose.position));

  // THEN: the histogram should be all zeros
  for (int e = 0; e < GRID_LENGTH_E; e++) {
    for (int z = 0; z < GRID_LENGTH_Z; z++) {
      EXPECT_LE(histogram_output.get_dist(e, z), FLT_MIN);
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
  float distance = 1.0f;

  std::vector<float> e_angle_filled = {-89.9, -30, 0, 20, 40, 89.9};
  std::vector<float> z_angle_filled = {-180, -50, 0, 59, 100, 175};
  std::vector<Eigen::Vector3f> middle_of_cell;
  std::vector<int> e_index, z_index;

  for (auto i : e_angle_filled) {
    for (auto j : z_angle_filled) {
      PolarPoint p_pol(i, j, distance);
      middle_of_cell.push_back(polarToCartesian(p_pol, location.pose.position));
      e_index.push_back(polarToHistogramIndex(p_pol, ALPHA_RES).y());
      z_index.push_back(polarToHistogramIndex(p_pol, ALPHA_RES).x());
    }
  }

  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (int i = 0; i < middle_of_cell.size(); i++) {
    // put 1000 point in every occupied cell
    for (int j = 0; j < 1000; j++) {
      cloud.push_back(toXYZ(middle_of_cell[i]));
    }
  }

  // WHEN: we build a histogram
  generateNewHistogram(histogram_output, cloud,
                       toEigen(location.pose.position));

  // THEN: the filled cells in the histogram should be one and the others be
  // zeros

  for (int e = 0; e < GRID_LENGTH_E; e++) {
    for (int z = 0; z < GRID_LENGTH_Z; z++) {
      bool e_found =
          std::find(e_index.begin(), e_index.end(), e) != e_index.end();
      bool z_found =
          std::find(z_index.begin(), z_index.end(), z) != z_index.end();
      if (e_found && z_found) {
        EXPECT_NEAR(histogram_output.get_dist(e, z), 1.f, 0.01);
      } else {
        EXPECT_LT(histogram_output.get_dist(e, z), FLT_MIN);
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
  float min_dist_backoff = 1.0f;
  Box histogram_box(5.0f);
  histogram_box.setBoxLimits(toPoint(position), 4.5f);
  float min_realsense_dist = 0.2f;

  pcl::PointCloud<pcl::PointXYZ> cropped_cloud, cropped_cloud2;
  Eigen::Vector3f closest_point, closest_point2;
  float distance_to_closest_point, distance_to_closest_point2;
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

TEST(PlannerFunctions, testDirectionTree) {
  // GIVEN: the node positions in a tree and some possible vehicle positions
  geometry_msgs::Point n0;
  n0.x = 0.0f;
  n0.y = 0.0f;
  n0.z = 2.5;
  geometry_msgs::Point n1;
  n1.x = 0.8f;
  n1.y = sqrtf(1 - (n1.x * n1.x));
  n1.z = 2.5;
  geometry_msgs::Point n2;
  n2.x = 1.5f;
  n2.y = n1.y + sqrtf(1 - powf(n2.x - n1.x, 2));
  n2.z = 2.5;
  geometry_msgs::Point n3;
  n3.x = 2.1f;
  n3.y = n2.y + sqrtf(1 - powf(n3.x - n2.x, 2));
  n3.z = 2.5;
  geometry_msgs::Point n4;
  n4.x = 2.3f;
  n4.y = n3.y + sqrtf(1 - powf(n4.x - n3.x, 2));
  n4.z = 2.5;
  const std::vector<geometry_msgs::Point> path_node_positions = {n4, n3, n2, n1,
                                                                 n0};

  PolarPoint p, p1, p2;
  Eigen::Vector3f postion(0.2, 0.3, 1.5);
  Eigen::Vector3f postion1(1.1, 2.3, 2.5);
  Eigen::Vector3f postion2(5.4, 2.0, 2.5);

  // WHEN: we look for the best direction to fly towards
  bool res = getDirectionFromTree(p, path_node_positions, postion);
  bool res1 = getDirectionFromTree(p1, path_node_positions, postion1);
  bool res2 = getDirectionFromTree(p2, path_node_positions, postion2);

  // THEN: we expect a direction in between node n1 and n2 for position, between
  // node n3 and n4 for position1, and not to get an available tree for the
  // position2
  ASSERT_TRUE(res);
  EXPECT_NEAR(45.0f, p.e, 1.f);
  EXPECT_NEAR(57.0f, p.z, 1.f);

  ASSERT_TRUE(res1);
  EXPECT_NEAR(0.0f, p1.e, 1.f);
  EXPECT_NEAR(72.0f, p1.z, 1.f);

  ASSERT_FALSE(res2);
}

TEST(PlannerFunctions, padPolarMatrixAzimuthWrapping) {
  // GIVEN: a matrix with known data. Where every cell has the value of its
  // column index.
  // And by how many lines should be padded
  int n_lines_padding = 3;
  Eigen::MatrixXf matrix;
  matrix.resize(GRID_LENGTH_E, GRID_LENGTH_Z);
  matrix.fill(1.0);
  for (int c = 0; c < matrix.cols(); c++) {
    matrix.col(c) = c * matrix.col(c);
  }

  // WHEN: we pad the matrix
  Eigen::MatrixXf matrix_padded;
  padPolarMatrix(matrix, n_lines_padding, matrix_padded);

  // THEN: the output matrix should have the right size,
  // the middle part should be equal to the original matrix,
  // and the wrapping around the azimuth angle should be correct.

  ASSERT_EQ(GRID_LENGTH_E + 2 * n_lines_padding, matrix_padded.rows());
  ASSERT_EQ(GRID_LENGTH_Z + 2 * n_lines_padding, matrix_padded.cols());

  bool middle_part_correct =
      matrix_padded.block(n_lines_padding, n_lines_padding, matrix.rows(),
                          matrix.cols()) == matrix;
  bool col_0_correct = matrix_padded.col(0) == matrix_padded.col(GRID_LENGTH_Z);
  bool col_1_correct =
      matrix_padded.col(1) == matrix_padded.col(GRID_LENGTH_Z + 1);
  bool col_2_correct =
      matrix_padded.col(2) == matrix_padded.col(GRID_LENGTH_Z + 2);
  bool col_63_correct =
      matrix_padded.col(GRID_LENGTH_Z + 3) == matrix_padded.col(3);
  bool col_64_correct =
      matrix_padded.col(GRID_LENGTH_Z + 4) == matrix_padded.col(4);
  bool col_65_correct =
      matrix_padded.col(GRID_LENGTH_Z + 5) == matrix_padded.col(5);

  EXPECT_TRUE(middle_part_correct);
  EXPECT_TRUE(col_0_correct);
  EXPECT_TRUE(col_1_correct);
  EXPECT_TRUE(col_2_correct);
  EXPECT_TRUE(col_63_correct);
  EXPECT_TRUE(col_64_correct);
  EXPECT_TRUE(col_65_correct);
}

TEST(PlannerFunctions, padPolarMatrixElevationWrapping) {
  // GIVEN: a matrix with known data. Where every cell has the value of its
  // column index.
  // And by how many lines should be padded
  int n_lines_padding = 2;
  Eigen::MatrixXf matrix;
  matrix.resize(GRID_LENGTH_E, GRID_LENGTH_Z);
  matrix.fill(0.0);
  int half_z = GRID_LENGTH_Z / 2;
  int last_z = GRID_LENGTH_Z - 1;
  int last_e = GRID_LENGTH_E - 1;

  matrix(0, 0) = 1;
  matrix(0, 1) = 2;
  matrix(1, 0) = 3;
  matrix(0, half_z) = 4;
  matrix(1, half_z + 1) = 5;
  matrix(0, last_z) = 6;
  matrix(last_e, 0) = 7;
  matrix(last_e - 1, half_z - 1) = 8;
  matrix(last_e, last_z) = 9;

  // WHEN: we pad the matrix
  Eigen::MatrixXf matrix_padded;
  padPolarMatrix(matrix, n_lines_padding, matrix_padded);

  // THEN: the output matrix should have the right size,
  // the middle part should be equal to the original matrix,
  // and the wrapping around the elevation angle should be correct.

  ASSERT_EQ(GRID_LENGTH_E + 2 * n_lines_padding, matrix_padded.rows());
  ASSERT_EQ(GRID_LENGTH_Z + 2 * n_lines_padding, matrix_padded.cols());

  bool middle_part_correct =
      matrix_padded.block(n_lines_padding, n_lines_padding, matrix.rows(),
                          matrix.cols()) == matrix;
  bool val_1 = matrix_padded(1, half_z + n_lines_padding) == 1;
  bool val_2 = matrix_padded(1, half_z + n_lines_padding + 1) == 2;
  bool val_3 = matrix_padded(0, half_z + n_lines_padding) == 3;
  bool val_4 = matrix_padded(1, 2) == 4;
  bool val_5 = matrix_padded(0, 3) == 5;
  bool val_6 = matrix_padded(1, half_z - 1 + n_lines_padding) == 6;
  bool val_7 = matrix_padded(last_e + 1 + n_lines_padding,
                             half_z + n_lines_padding) == 7;
  bool val_8 = matrix_padded(last_e + 2 + n_lines_padding,
                             GRID_LENGTH_Z + n_lines_padding - 1) == 8;
  bool val_9 = matrix_padded(last_e + 1 + n_lines_padding,
                             half_z - 1 + n_lines_padding) == 9;

  EXPECT_TRUE(middle_part_correct);
  EXPECT_TRUE(val_1);
  EXPECT_TRUE(val_2);
  EXPECT_TRUE(val_3);
  EXPECT_TRUE(val_4);
  EXPECT_TRUE(val_5);
  EXPECT_TRUE(val_6);
  EXPECT_TRUE(val_7);
  EXPECT_TRUE(val_8);
  EXPECT_TRUE(val_9);
}

TEST(PlannerFunctions, getBestCandidatesFromCostMatrix) {
  // GIVEN: a known cost matrix and the number of needed candidates
  int n_candidates = 4;
  std::vector<candidateDirection> candidate_vector;
  Eigen::MatrixXf matrix;
  matrix.resize(GRID_LENGTH_E, GRID_LENGTH_Z);
  matrix.fill(10);

  matrix(0, 2) = 1.1;
  matrix(0, 1) = 2.5;
  matrix(1, 2) = 3.8;
  matrix(1, 0) = 4.7;
  matrix(2, 2) = 4.9;

  // WHEN: calculate the candidates from the matrix
  getBestCandidatesFromCostMatrix(matrix, n_candidates, candidate_vector);

  // THEN: the output vector should have the right candidates in the right order

  ASSERT_EQ(n_candidates, candidate_vector.size());

  EXPECT_FLOAT_EQ(1.1, candidate_vector[0].cost);
  EXPECT_FLOAT_EQ(2.5, candidate_vector[1].cost);
  EXPECT_FLOAT_EQ(3.8, candidate_vector[2].cost);
  EXPECT_FLOAT_EQ(4.7, candidate_vector[3].cost);
}

TEST(PlannerFunctions, smoothPolarMatrix) {
  // GIVEN: a smoothing radius and a known cost matrix with one costly cell,
  // otherwise all zeros.
  unsigned int smooth_radius = 2;
  Eigen::MatrixXf matrix;
  Eigen::MatrixXf matrix_old;
  matrix.resize(GRID_LENGTH_E, GRID_LENGTH_Z);
  matrix.fill(0);

  int r_object = GRID_LENGTH_E / 2;
  int c_object = GRID_LENGTH_Z / 2;
  matrix(r_object, c_object) = 100;

  // WHEN: we calculate the smoothed matrix
  matrix_old = matrix;
  smoothPolarMatrix(matrix, smooth_radius);

  // THEN: The smoothed matrix should be element-wise greater-equal than the
  // input matrix
  // and the elements inside the smoothing radius around the costly cell should
  // be greater than before
  for (int r = r_object - smooth_radius; r < r_object + smooth_radius; r++) {
    for (int c = c_object - smooth_radius; c < c_object + smooth_radius; c++) {
      if (!(r == r_object && c == c_object)) {
        EXPECT_GT(matrix(r, c), matrix_old(r, c));
      }
    }
  }
  bool greater_equal = (matrix.array() >= matrix_old.array()).all();
  EXPECT_TRUE(greater_equal);
}

TEST(PlannerFunctions, getCostMatrixNoObstacles) {
  // GIVEN: a position, goal and an empty histogram
  Eigen::Vector3f position(0, 0, 0);
  Eigen::Vector3f goal(0, 5, 0);
  Eigen::Vector3f last_sent_waypoint(0, 1, 0);
  costParameters cost_params;
  cost_params.goal_cost_param = 2.f;
  cost_params.smooth_cost_param = 1.5f;
  cost_params.height_change_cost_param = 4.f;
  cost_params.height_change_cost_param_adapted = 4.f;
  Eigen::MatrixXf cost_matrix;
  Histogram histogram = Histogram(ALPHA_RES);

  // WHEN: we calculate the cost matrix from the input data
  getCostMatrix(histogram, goal, position, last_sent_waypoint, cost_params,
                false, cost_matrix);

  // THEN: The minimum cost should be in the direction of the goal
  PolarPoint best_pol = cartesianToPolar(goal, position);
  Eigen::Vector2i best_index = polarToHistogramIndex(best_pol, ALPHA_RES);

  Eigen::MatrixXf::Index minRow, minCol;
  float min = cost_matrix.minCoeff(&minRow, &minCol);

  EXPECT_NEAR((int)minRow, best_index.y(), 1);
  EXPECT_NEAR((int)minCol, best_index.x(), 1);

  // And the cost should grow as we go away from the goal index
  int check_radius = 3;
  Eigen::MatrixXf matrix_padded;
  // extend the matrix, as the goal direction might be at the border
  padPolarMatrix(cost_matrix, check_radius, matrix_padded);

  int best_index_padded_e = (int)minRow + check_radius;
  int best_index_padded_z = (int)minCol + check_radius;

  // check that rows farther away have bigger cost. Leave out direct neighbor
  // rows as the center might be split over cells
  bool row1 = (matrix_padded.row(best_index_padded_e + 2).array() >
               matrix_padded.row(best_index_padded_e + 1).array())
                  .all();
  bool row2 = (matrix_padded.row(best_index_padded_e + 3).array() >
               matrix_padded.row(best_index_padded_e + 2).array())
                  .all();
  bool row3 = (matrix_padded.row(best_index_padded_e - 2).array() >
               matrix_padded.row(best_index_padded_e).array() - 1)
                  .all();
  bool row4 = (matrix_padded.row(best_index_padded_e - 3).array() >
               matrix_padded.row(best_index_padded_e).array() - 2)
                  .all();

  // check that columns farther away have bigger cost. Leave out direct neighbor
  // rows as the center might be split over cells
  Eigen::MatrixXf matrix_padded2 =
      matrix_padded.block(3, 0, cost_matrix.rows(),
                          matrix_padded.cols());  // cut off padded top part
  bool col1 = (matrix_padded2.col(best_index_padded_z + 2).array() >
               matrix_padded2.col(best_index_padded_z + 1).array())
                  .all();
  bool col2 = (matrix_padded2.col(best_index_padded_z + 3).array() >
               matrix_padded2.col(best_index_padded_z + 2).array())
                  .all();
  bool col3 = (matrix_padded2.col(best_index_padded_z - 2).array() >
               matrix_padded2.col(best_index_padded_z).array() - 1)
                  .all();
  bool col4 = (matrix_padded2.col(best_index_padded_z - 3).array() >
               matrix_padded2.col(best_index_padded_z).array() - 2)
                  .all();

  EXPECT_TRUE(col1);
  EXPECT_TRUE(col2);
  EXPECT_TRUE(col3);
  EXPECT_TRUE(col4);
  EXPECT_TRUE(row1);
  EXPECT_TRUE(row2);
  EXPECT_TRUE(row3);
  EXPECT_TRUE(row4);
}

TEST(Histogram, HistogramDownsampleCorrectUsage) {
  // GIVEN: a histogram of the correct resolution
  Histogram histogram = Histogram(ALPHA_RES);
  histogram.set_dist(0, 0, 1.3);
  histogram.set_dist(1, 0, 1.3);
  histogram.set_dist(0, 1, 1.3);
  histogram.set_dist(1, 1, 1.3);
  histogram.set_age(2, 2, 3);
  histogram.set_age(3, 2, 3);
  histogram.set_age(2, 3, 3);
  histogram.set_age(3, 3, 3);

  // WHEN: we downsample the histogram to have a larger bin size
  histogram.downsample();

  // THEN: The downsampled histogram should fuse four cells of the regular
  // resolution histogram into one
  for (int i = 0; i < GRID_LENGTH_E / 2; ++i) {
    for (int j = 0; j < GRID_LENGTH_Z / 2; ++j) {
      if (i == 0 && j == 0) {
        EXPECT_FLOAT_EQ(1.3, histogram.get_dist(i, j));
        EXPECT_FLOAT_EQ(0.0, histogram.get_age(i, j));
      } else if (i == 1 && j == 1) {
        EXPECT_FLOAT_EQ(3, histogram.get_age(i, j));
        EXPECT_FLOAT_EQ(0.0, histogram.get_dist(i, j));
      } else {
        EXPECT_FLOAT_EQ(0.0, histogram.get_dist(i, j));
        EXPECT_FLOAT_EQ(0.0, histogram.get_age(i, j));
      }
    }
  }
}

TEST(Histogram, HistogramUpsampleCorrectUsage) {
  // GIVEN: a histogram of the correct resolution
  Histogram histogram = Histogram(ALPHA_RES * 2);
  histogram.set_dist(0, 0, 1.3);
  histogram.set_age(1, 1, 3);

  // WHEN: we upsample the histogram to have regular bin size
  histogram.upsample();

  // THEN: The upsampled histogram should split every cell of the lower
  // resolution histogram into four cells
  for (int i = 0; i < GRID_LENGTH_E; ++i) {
    for (int j = 0; j < GRID_LENGTH_Z; ++j) {
      if ((i == 0 && j == 0) || (i == 1 && j == 0) || (i == 0 && j == 1) ||
          (i == 1 && j == 1)) {
        EXPECT_FLOAT_EQ(1.3, histogram.get_dist(i, j));
        EXPECT_FLOAT_EQ(0.0, histogram.get_age(i, j));
      } else if ((i == 2 && j == 2) || (i == 2 && j == 3) ||
                 (i == 3 && j == 2) || (i == 3 && j == 3)) {
        EXPECT_FLOAT_EQ(3, histogram.get_age(i, j));
        EXPECT_FLOAT_EQ(0.0, histogram.get_dist(i, j));
      } else {
        EXPECT_FLOAT_EQ(0.0, histogram.get_dist(i, j));
        EXPECT_FLOAT_EQ(0.0, histogram.get_age(i, j));
      }
    }
  }
}

TEST(Histogram, HistogramUpDownpsampleInorrectUsage) {
  // GIVEN: a histogram of the correct resolution
  Histogram low_res_histogram = Histogram(ALPHA_RES * 2);
  Histogram high_res_histogram = Histogram(ALPHA_RES);

  // THEN: We expect the functions to throw if the input histogram has the wrong
  // resolution
  EXPECT_THROW(low_res_histogram.downsample(), std::logic_error);
  EXPECT_THROW(high_res_histogram.upsample(), std::logic_error);
}
