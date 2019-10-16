#include <gtest/gtest.h>
#include <cmath>

#include "../include/local_planner/planner_functions.h"

#include "avoidance/common.h"

using namespace avoidance;

TEST(PlannerFunctions, generateNewHistogramEmpty) {
  // GIVEN: an empty pointcloud
  pcl::PointCloud<pcl::PointXYZI> empty_cloud;
  Histogram histogram_output = Histogram(ALPHA_RES);
  geometry_msgs::PoseStamped location;
  location.pose.position.x = 0;
  location.pose.position.y = 0;
  location.pose.position.z = 0;

  // WHEN: we build a histogram
  generateNewHistogram(histogram_output, empty_cloud, toEigen(location.pose.position));

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
  Eigen::Vector3f location(0.0f, 0.0f, 0.0f);
  float distance = 1.0f;

  std::vector<float> e_angle_filled = {-89.9f, -30.0f, 0.0f, 20.0f, 40.0f, 89.9f};
  std::vector<float> z_angle_filled = {-180.0f, -50.0f, 0.0f, 59.0f, 100.0f, 175.0f};
  std::vector<Eigen::Vector3f> middle_of_cell;
  std::vector<int> e_index, z_index;

  for (auto i : e_angle_filled) {
    for (auto j : z_angle_filled) {
      PolarPoint p_pol(i, j, distance);
      middle_of_cell.push_back(polarHistogramToCartesian(p_pol, location));
      e_index.push_back(polarToHistogramIndex(p_pol, ALPHA_RES).y());
      z_index.push_back(polarToHistogramIndex(p_pol, ALPHA_RES).x());
    }
  }

  pcl::PointCloud<pcl::PointXYZI> cloud;
  for (int i = 0; i < middle_of_cell.size(); i++) {
    // put 1000 point in every occupied cell
    for (int j = 0; j < 1000; j++) {
      cloud.push_back(toXYZI(middle_of_cell[i], 0));
    }
  }

  // WHEN: we build a histogram
  generateNewHistogram(histogram_output, cloud, location);

  // THEN: the filled cells in the histogram should be one and the others be
  // zeros

  for (int e = 0; e < GRID_LENGTH_E; e++) {
    for (int z = 0; z < GRID_LENGTH_Z; z++) {
      bool e_found = std::find(e_index.begin(), e_index.end(), e) != e_index.end();
      bool z_found = std::find(z_index.begin(), z_index.end(), z) != z_index.end();
      if (e_found && z_found) {
        EXPECT_NEAR(histogram_output.get_dist(e, z), 1.f, 0.01);
      } else {
        EXPECT_LT(histogram_output.get_dist(e, z), FLT_MIN);
      }
    }
  }
}

TEST(PlannerFunctionsTests, processPointcloud) {
  // GIVEN: two point clouds
  const Eigen::Vector3f position(1.5f, 1.0f, 4.5f);
  pcl::PointCloud<pcl::PointXYZ> p1;
  p1.push_back(toXYZ(position + Eigen::Vector3f(1.1f, 0.8f, 0.1f)));
  p1.push_back(toXYZ(position + Eigen::Vector3f(2.2f, 1.0f, 1.0f)));
  p1.push_back(toXYZ(position + Eigen::Vector3f(1.0f, -3.0f, 1.0f)));
  p1.push_back(toXYZ(position + Eigen::Vector3f(0.7f, 0.3f, -0.5f)));
  p1.push_back(toXYZ(position + Eigen::Vector3f(-1.0f, 1.0f, 1.0f)));
  p1.push_back(toXYZ(position + Eigen::Vector3f(-1.0f, -1.1f, 3.5f)));

  pcl::PointCloud<pcl::PointXYZ> p2;
  p2.push_back(toXYZ(position + Eigen::Vector3f(1.0f, 5.0f, 1.0f)));
  p2.push_back(toXYZ(position + Eigen::Vector3f(100.0f, 5.0f, 1.0f)));  // > max_sensor_dist
  p2.push_back(toXYZ(position + Eigen::Vector3f(0.1f, 0.05f, 0.05f)));  // < min_sensor_dist

  std::vector<pcl::PointCloud<pcl::PointXYZ>> complete_cloud;
  complete_cloud.push_back(p1);
  complete_cloud.push_back(p2);
  float min_sensor_dist = 0.2f;
  float max_sensor_dist = 12.f;

  pcl::PointCloud<pcl::PointXYZI> processed_cloud1, processed_cloud2, processed_cloud3;
  Eigen::Vector3f memory_point(1.4f, 0.0f, 0.0f);
  PolarPoint memory_point_polar = cartesianToPolarFCU(position + memory_point, position);
  processed_cloud1.push_back(toXYZI(position + memory_point, 5.0f));
  processed_cloud2.push_back(toXYZI(position + memory_point, 5.0f));
  processed_cloud3.push_back(toXYZI(position + memory_point, 5.0f));

  std::vector<FOV> FOV_zero;  // zero FOV means all pts are outside FOV, and thus remembered
  FOV_zero.push_back(FOV(1.0f, 1.0f, 0.0f, 0.0f));

  std::vector<FOV> FOV_regular;
  FOV_regular.push_back(FOV(0.0f, 1.0f, 85.f, 65.f));

  // WHEN: we filter the PointCloud with different values max_age
  processPointcloud(processed_cloud1, complete_cloud, FOV_zero, 0.0f, 0.0f, position, min_sensor_dist, max_sensor_dist,
                    0.0f, 0.5f, 1);

  // todo: test different yaw and pitch
  processPointcloud(processed_cloud2, complete_cloud, FOV_zero, 0.0f, 0.0f, position, min_sensor_dist, max_sensor_dist,
                    10.0f, .5f, 1);

  processPointcloud(processed_cloud3, complete_cloud, FOV_regular, 0.0f, 0.0f, position, min_sensor_dist,
                    max_sensor_dist, 10.0f, 0.5f, 1);

  // THEN: we expect the first cloud to have 5 points
  // the second cloud should contain all 6 points
  EXPECT_EQ(7, processed_cloud1.size());
  EXPECT_EQ(8, processed_cloud2.size());
  EXPECT_TRUE(pointInsideFOV(FOV_regular, memory_point_polar));
  EXPECT_EQ(7, processed_cloud3.size());  // since memory point is inside FOV, it isn't remembered
}

TEST(PlannerFunctions, compressHistogramElevation) {
  // GIVEN: a position and a pointcloud with data
  const Eigen::Vector3f position(0.f, 0.f, 5.f);
  const Eigen::Vector3f data_point(14.f, 0.f, 5.f);
  pcl::PointCloud<pcl::PointXYZI> cloud;

  for (int j = 0; j < 1000; j++) {
    cloud.push_back(toXYZI(data_point, 0));
  }

  Histogram histogram = Histogram(ALPHA_RES);
  Histogram compressed_histogram = Histogram(ALPHA_RES);

  // WHEN: we build a histogram
  generateNewHistogram(histogram, cloud, position);
  compressHistogramElevation(compressed_histogram, histogram, position);

  PolarPoint data_pol = cartesianToPolarHistogram(data_point, position);
  Eigen::Vector2i indices = polarToHistogramIndex(data_pol, ALPHA_RES);

  for (int z = 0; z < GRID_LENGTH_Z; z++) {
    if (z == indices.x()) {
      EXPECT_FLOAT_EQ(compressed_histogram.get_dist(0, z), 14.f);
    } else {
      EXPECT_FLOAT_EQ(compressed_histogram.get_dist(0, z), 0.f);
    }
  }
}

TEST(PlannerFunctions, getSetpointFromPath) {
  // GIVEN: the node positions in a path and some possible vehicle positions
  float n1_x = 0.8f;
  float n2_x = 1.5f;
  float n3_x = 2.1f;
  float n4_x = 2.3f;
  Eigen::Vector3f n0(0.0f, 0.0f, 2.5f);
  Eigen::Vector3f n1(n1_x, sqrtf(1 - (n1_x * n1_x)), 2.5f);
  Eigen::Vector3f n2(n2_x, n1.y() + sqrtf(1 - powf(n2_x - n1.x(), 2)), 2.5f);
  Eigen::Vector3f n3(n3_x, n2.y() + sqrtf(1 - powf(n3_x - n2.x(), 2)), 2.5f);
  Eigen::Vector3f n4(n4_x, n3.y() + sqrtf(1 - powf(n4_x - n3.x(), 2)), 2.5f);
  const std::vector<Eigen::Vector3f> path_node_positions = {n4, n3, n2, n1, n0};
  const std::vector<Eigen::Vector3f> empty_path = {};
  ros::Time t1 = ros::Time::now();
  ros::Time t2 = t1 - ros::Duration(0.1);
  ros::Time t3 = t1 - ros::Duration(1.1);

  float velocity = 1.0f;  // assume we're flying at 1m/s

  Eigen::Vector3f sp1, sp2, sp3;

  // WHEN: we look for the best direction to fly towards
  bool res = getSetpointFromPath(path_node_positions, t1, velocity, ros::Time::now(),
                                 sp1);  // very short time should still return node 1
  bool res1 = getSetpointFromPath(path_node_positions, t2, velocity, ros::Time::now(), sp2);
  bool res2 =
      getSetpointFromPath(path_node_positions, t3, velocity, ros::Time::now(), sp3);  // should be second node on path
  bool res3 = getSetpointFromPath(empty_path, t1, velocity, ros::Time::now(), sp1);

  // THEN: we expect the setpoint in between node n1 and n2 for t1 and t2 between
  // node n2 and n3 for t3, and not to get an available path for the empty path
  ASSERT_TRUE(res);
  EXPECT_GE(sp1.x(), n1.x());
  EXPECT_LE(sp1.x(), n2.x());
  EXPECT_GE(sp1.y(), n1.y());
  EXPECT_LE(sp1.y(), n2.y());
  EXPECT_GE(sp1.z(), n1.z());
  EXPECT_LE(sp1.z(), n2.z());

  ASSERT_TRUE(res1);
  EXPECT_GE(sp2.x(), n1.x());
  EXPECT_LE(sp2.x(), n2.x());
  EXPECT_GE(sp2.y(), n1.y());
  EXPECT_LE(sp2.y(), n2.y());
  EXPECT_GE(sp2.z(), n1.z());
  EXPECT_LE(sp2.z(), n2.z());

  ASSERT_TRUE(res2);
  EXPECT_GE(sp3.x(), n2.x());
  EXPECT_LE(sp3.x(), n3.x());
  EXPECT_GE(sp3.y(), n2.y());
  EXPECT_LE(sp3.y(), n3.y());
  EXPECT_GE(sp3.z(), n2.z());
  EXPECT_LE(sp3.z(), n3.z());

  ASSERT_FALSE(res3);
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
      matrix_padded.block(n_lines_padding, n_lines_padding, matrix.rows(), matrix.cols()) == matrix;
  bool col_0_correct = matrix_padded.col(0) == matrix_padded.col(GRID_LENGTH_Z);
  bool col_1_correct = matrix_padded.col(1) == matrix_padded.col(GRID_LENGTH_Z + 1);
  bool col_2_correct = matrix_padded.col(2) == matrix_padded.col(GRID_LENGTH_Z + 2);
  bool col_63_correct = matrix_padded.col(GRID_LENGTH_Z + 3) == matrix_padded.col(3);
  bool col_64_correct = matrix_padded.col(GRID_LENGTH_Z + 4) == matrix_padded.col(4);
  bool col_65_correct = matrix_padded.col(GRID_LENGTH_Z + 5) == matrix_padded.col(5);

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
      matrix_padded.block(n_lines_padding, n_lines_padding, matrix.rows(), matrix.cols()) == matrix;
  bool val_1 = matrix_padded(1, half_z + n_lines_padding) == 1;
  bool val_2 = matrix_padded(1, half_z + n_lines_padding + 1) == 2;
  bool val_3 = matrix_padded(0, half_z + n_lines_padding) == 3;
  bool val_4 = matrix_padded(1, 2) == 4;
  bool val_5 = matrix_padded(0, 3) == 5;
  bool val_6 = matrix_padded(1, half_z - 1 + n_lines_padding) == 6;
  bool val_7 = matrix_padded(last_e + 1 + n_lines_padding, half_z + n_lines_padding) == 7;
  bool val_8 = matrix_padded(last_e + 2 + n_lines_padding, GRID_LENGTH_Z + n_lines_padding - 1) == 8;
  bool val_9 = matrix_padded(last_e + 1 + n_lines_padding, half_z - 1 + n_lines_padding) == 9;

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

TEST(PlannerFunctions, smoothMatrix) {
  // GIVEN: a matrix with a single cell set
  unsigned int smooth_radius = 4;
  Eigen::MatrixXf matrix;
  matrix.resize(10, 20);
  matrix.fill(0);
  matrix(3, 16) = 100;
  matrix(6, 6) = -100;

  // WHEN: we smooth it
  Eigen::MatrixXf matrix_old = matrix;
  smoothPolarMatrix(matrix, smooth_radius);

  // THEN: it should match the matrix we expect
  Eigen::MatrixXf expected_matrix(matrix.rows(), matrix.cols());
  // clang-format off
  expected_matrix <<
   8, 0,   4,   8,  12,  16,  20,  16,  12,   8,   4, 0,  8, 16,  24,  32,  40,  32,  24, 16,
  12, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0, 12, 24,  36,  48,  60,  48,  36, 24,
  16, 0,  -4,  -8, -12, -16, -20, -16, -12,  -8,  -4, 0, 16, 32,  48,  64,  80,  64,  48, 32,
  20, 0,  -8, -16, -24, -32, -40, -32, -24, -16,  -8, 0, 20, 40,  60,  80, 100,  80,  60, 40,
  16, 0, -12, -24, -36, -48, -60, -48, -36, -24, -12, 0, 16, 32,  48,  64,  80,  64,  48, 32,
  12, 0, -16, -32, -48, -64, -80, -64, -48, -32, -16, 0, 12, 24,  36,  48,  60,  48,  36, 24,
   8, 0, -20, -40, -60, -80,-100, -80, -60, -40, -20, 0,  8, 16,  24,  32,  40,  32,  24, 16,
   4, 0, -16, -32, -48, -64, -80, -64, -48, -32, -16, 0,  4,  8,  12,  16,  20,  16,  12,  8,
   0, 0, -12, -24, -36, -48, -60, -48, -36, -24, -12, 0,  0,  0,   0,   0,   0,   0,   0,  0,
  -4, 0,  -8, -16, -24, -32, -40, -32, -24, -16,  -8, 0, -4, -8, -12, -16, -20, -16, -12, -8;
  // clang-format on

  EXPECT_LT((expected_matrix - matrix).cwiseAbs().maxCoeff(), 1e-5);
}

TEST(PlannerFunctions, getCostMatrixNoObstacles) {
  // GIVEN: a position, goal and an empty histogram
  Eigen::Vector3f position(0.f, 0.f, 0.f);
  Eigen::Vector3f velocity(1.f, 0.f, 0.f);  // despite some initial velocity orthogonal to the goal!
  Eigen::Vector3f goal(0.f, 5.f, 0.f);
  costParameters cost_params;
  cost_params.yaw_cost_param = 2.5f;
  cost_params.pitch_cost_param = 10.0f;
  cost_params.velocity_cost_param = 1200.f;
  cost_params.obstacle_cost_param = 5.0f;
  Eigen::MatrixXf cost_matrix;
  Histogram histogram = Histogram(ALPHA_RES);
  float smoothing_radius = 30.f;
  const float max_sensor_range = 15.f;
  const float min_sensor_range = 0.2f;

  // WHEN: we calculate the cost matrix from the input data
  std::vector<uint8_t> cost_image_data;
  getCostMatrix(histogram, goal, position, velocity, cost_params, smoothing_radius, goal, max_sensor_range,
                min_sensor_range, cost_matrix, cost_image_data);

  // THEN: The minimum cost should be in the direction of the goal
  PolarPoint best_pol = cartesianToPolarHistogram(goal, position);
  Eigen::Vector2i best_index = polarToHistogramIndex(best_pol, ALPHA_RES);

  Eigen::MatrixXf::Index minRow, minCol;
  cost_matrix.minCoeff(&minRow, &minCol);

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
  bool row1 =
      (matrix_padded.row(best_index_padded_e + 2).array() > matrix_padded.row(best_index_padded_e + 1).array()).all();
  bool row2 =
      (matrix_padded.row(best_index_padded_e + 3).array() > matrix_padded.row(best_index_padded_e + 2).array()).all();
  bool row3 =
      (matrix_padded.row(best_index_padded_e - 2).array() > matrix_padded.row(best_index_padded_e).array() - 1).all();
  bool row4 =
      (matrix_padded.row(best_index_padded_e - 3).array() > matrix_padded.row(best_index_padded_e).array() - 2).all();

  // check that columns farther away have bigger cost. Leave out direct neighbor
  // rows as the center might be split over cells
  Eigen::MatrixXf matrix_padded2 = matrix_padded.block(3, 3, cost_matrix.rows(),
                                                       cost_matrix.cols());  // cut off padded top part
  bool col1 =
      (matrix_padded2.col(best_index_padded_z + 10).array() > matrix_padded2.col(best_index_padded_z + 1).array())
          .all();
  bool col2 =
      (matrix_padded2.col(best_index_padded_z + 20).array() > matrix_padded2.col(best_index_padded_z + 10).array())
          .all();
  bool col3 =
      (matrix_padded2.col(best_index_padded_z - 10).array() > matrix_padded2.col(best_index_padded_z).array() - 1)
          .all();
  bool col4 =
      (matrix_padded2.col(best_index_padded_z - 20).array() > matrix_padded2.col(best_index_padded_z).array() - 10)
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

TEST(PlannerFunctions, CostfunctionGoalCost) {
  // GIVEN: a scenario with two different goal locations
  Eigen::Vector3f position(0.f, 0.f, 0.f);
  Eigen::Vector3f velocity(0.f, 0.f, 0.f);
  Eigen::Vector3f goal_1(0.f, 5.f, 0.f);
  Eigen::Vector3f goal_2(3.f, 3.f, 0.f);

  costParameters cost_params;
  cost_params.yaw_cost_param = 2.5f;
  cost_params.pitch_cost_param = 10.0f;
  cost_params.velocity_cost_param = 1200.f;
  cost_params.obstacle_cost_param = 5.0f;
  float obstacle_distance = 0.f;
  std::pair<float, float> cost_1, cost_2;

  PolarPoint candidate_1(0.f, 0.f, 1.0f);

  // WHEN: we calculate the cost of one cell for the same scenario but with two
  // different goals
  cost_1 = costFunction(candidate_1, obstacle_distance, goal_1, position, velocity, cost_params, goal_1, true);
  cost_2 = costFunction(candidate_1, obstacle_distance, goal_2, position, velocity, cost_params, goal_2, true);

  // THEN: The cost in the case where the goal is in the cell direction should
  // be lower
  EXPECT_LT(cost_1.second, cost_2.second);
}

TEST(PlannerFunctions, CostfunctionDistanceCost) {
  // GIVEN: a scenario with two different obstacle distances
  Eigen::Vector3f position(0.f, 0.f, 0.f);
  Eigen::Vector3f velocity(0.f, 0.f, 0.f);
  Eigen::Vector3f goal(0.f, 5.f, 0.f);

  costParameters cost_params;
  cost_params.yaw_cost_param = 2.5f;
  cost_params.pitch_cost_param = 10.0f;
  cost_params.velocity_cost_param = 1200.f;
  cost_params.obstacle_cost_param = 5.0f;
  float distance_1 = 0.f;
  float distance_2 = 3.f;
  float distance_3 = 5.f;
  std::pair<float, float> cost_1, cost_2, cost_3;
  PolarPoint candidate_1(0.f, 0.f, 1.0f);

  // WHEN: we calculate the cost of one cell for the same scenario but with two
  // different obstacle distance
  cost_1 = costFunction(candidate_1, distance_1, goal, position, velocity, cost_params, goal, true);
  cost_2 = costFunction(candidate_1, distance_2, goal, position, velocity, cost_params, goal, true);
  cost_3 = costFunction(candidate_1, distance_3, goal, position, velocity, cost_params, goal, true);

  // THEN: The distance cost for no obstacle should be zero and the distance
  // cost for the closer obstacle should be bigger
  EXPECT_LT(cost_1.first, cost_2.first);
  EXPECT_LT(cost_3.first, cost_2.first);
  EXPECT_FLOAT_EQ(cost_1.first, 0.f);
}

TEST(PlannerFunctions, CostfunctionVelocityCost) {
  // GIVEN: a scenario with two different initial headings
  Eigen::Vector3f position(0.f, 0.f, 0.f);
  Eigen::Vector3f velocity_1(0.f, 1.f, 0.f);
  Eigen::Vector3f velocity_2(1.f, 0.f, 0.f);
  Eigen::Vector3f goal(0.f, 5.f, 0.f);

  costParameters cost_params;
  cost_params.yaw_cost_param = 2.5f;
  cost_params.pitch_cost_param = 10.0f;
  cost_params.velocity_cost_param = 1200.f;
  cost_params.obstacle_cost_param = 5.0f;
  float obstacle_distance = 0.f;
  std::pair<float, float> cost_1, cost_2;

  PolarPoint candidate_1(0.f, 0.f, 1.0f);

  // WHEN: we calculate the cost of one cell for the same scenario but with two
  // different initial velocities
  cost_1 = costFunction(candidate_1, obstacle_distance, goal, position, velocity_1, cost_params, goal, true);
  cost_2 = costFunction(candidate_1, obstacle_distance, goal, position, velocity_2, cost_params, goal, true);

  // THEN: The cost in the case where the initial heading is closer to the
  // candidate should be lower
  EXPECT_LT(cost_1.second, cost_2.second);
}

TEST(Histogram, HistogramDownsampleCorrectUsage) {
  // GIVEN: a histogram of the correct resolution
  Histogram histogram = Histogram(ALPHA_RES);
  histogram.set_dist(0, 0, 1.3);
  histogram.set_dist(1, 0, 1.3);
  histogram.set_dist(0, 1, 1.3);
  histogram.set_dist(1, 1, 1.3);

  // WHEN: we downsample the histogram to have a larger bin size
  histogram.downsample();

  // THEN: The downsampled histogram should fuse four cells of the regular
  // resolution histogram into one
  for (int i = 0; i < GRID_LENGTH_E / 2; ++i) {
    for (int j = 0; j < GRID_LENGTH_Z / 2; ++j) {
      if (i == 0 && j == 0) {
        EXPECT_FLOAT_EQ(1.3, histogram.get_dist(i, j));
      } else if (i == 1 && j == 1) {
        EXPECT_FLOAT_EQ(0.0, histogram.get_dist(i, j));
      } else {
        EXPECT_FLOAT_EQ(0.0, histogram.get_dist(i, j));
      }
    }
  }
}

TEST(Histogram, HistogramUpsampleCorrectUsage) {
  // GIVEN: a histogram of the correct resolution
  Histogram histogram = Histogram(ALPHA_RES * 2);
  histogram.set_dist(0, 0, 1.3);

  // WHEN: we upsample the histogram to have regular bin size
  histogram.upsample();

  // THEN: The upsampled histogram should split every cell of the lower
  // resolution histogram into four cells
  for (int i = 0; i < GRID_LENGTH_E; ++i) {
    for (int j = 0; j < GRID_LENGTH_Z; ++j) {
      if ((i == 0 && j == 0) || (i == 1 && j == 0) || (i == 0 && j == 1) || (i == 1 && j == 1)) {
        EXPECT_FLOAT_EQ(1.3, histogram.get_dist(i, j));
      } else if ((i == 2 && j == 2) || (i == 2 && j == 3) || (i == 3 && j == 2) || (i == 3 && j == 3)) {
        EXPECT_FLOAT_EQ(0.0, histogram.get_dist(i, j));
      } else {
        EXPECT_FLOAT_EQ(0.0, histogram.get_dist(i, j));
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

TEST(Histogram, HistogramisEmpty) {
  // GIVEN: a histogram
  Histogram histogram = Histogram(ALPHA_RES);
  // Set a cell
  histogram.set_dist(0, 0, 1.3);
  EXPECT_FALSE(histogram.isEmpty());

  // Set the cell dist to 0.f
  histogram.set_dist(0, 0, 0.f);
  EXPECT_TRUE(histogram.isEmpty());
}
