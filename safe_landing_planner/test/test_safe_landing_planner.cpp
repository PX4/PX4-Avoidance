#include <gtest/gtest.h>

#include "../include/safe_landing_planner/safe_landing_planner.hpp"
#include "../include/safe_landing_planner/safe_landing_planner_node.hpp"

#include <chrono>
#include <fstream>
#include <iostream>
#include <numeric>
#include <random>
#include <sstream>
#include <vector>

using namespace avoidance;

class TestSafeLanding : public ::SafeLandingPlanner {
 public:
  Grid& test_getGrid() { return grid_; }
  Grid& test_getPrevGrid() { return previous_grid_; }
  float& test_getCounterThreshold() { return n_points_thr_; }
};

class SafeLandingPlannerTests : public ::testing::Test {
 public:
  TestSafeLanding safe_landing_planner;
  Eigen::Quaternionf q;
  unsigned seed = 10;

  void SetUp() override {
    ros::Time::init();
    Eigen::Vector3f pos(4.2f, 3.9f, 5.f);
    q = Eigen::Quaternionf(1.f, 0.f, 0.f, 0.f);
    safe_landing_planner.setPose(pos, q);

    safe_landing_planner::SafeLandingPlannerNodeConfig config =
        safe_landing_planner::SafeLandingPlannerNodeConfig::__getDefault__();
    safe_landing_planner.dynamicReconfigureSetParams(config, 1);
  }
  void TearDown() override {}
};

TEST_F(SafeLandingPlannerTests, flat_center) {
  safe_landing_planner::SafeLandingPlannerNodeConfig config =
      safe_landing_planner::SafeLandingPlannerNodeConfig::__getDefault__();

  config.smoothing_size = -1;
  config.n_points_threshold = 20;
  config.min_n_land_cells = 20;
  config.cell_size = 1;
  config.alpha = 0.0;

  safe_landing_planner.dynamicReconfigureSetParams(config, 1);

  // create pointcloud with 3x3m flat area in the center
  std::default_random_engine generator(seed);
  std::normal_distribution<float> distribution(0.0f, 1.0f);

  Eigen::Vector3f pos(5.f, 5.f, 5.f);
  safe_landing_planner.setPose(pos, q);
  std::uniform_real_distribution<float> uniform_distribution_0_9(.0f, 9.f);
  std::uniform_real_distribution<float> uniform_distribution_0_3(.0f, 3.f);
  std::uniform_real_distribution<float> uniform_distribution_3_6(3.f, 6.f);
  std::uniform_real_distribution<float> uniform_distribution_6_9(6.f, 9.f);

  for (int i = 0; i < 1000; ++i) {
    float x = uniform_distribution_0_9(generator);
    float y = uniform_distribution_0_3(generator);
    safe_landing_planner.cloud_.push_back(pcl::PointXYZ(x, y, distribution(generator)));
  }

  for (int i = 0; i < 1000; ++i) {
    float x = uniform_distribution_0_9(generator);
    float y = uniform_distribution_6_9(generator);
    safe_landing_planner.cloud_.push_back(pcl::PointXYZ(x, y, distribution(generator)));
  }

  for (int i = 0; i < 500; ++i) {
    float x = uniform_distribution_0_3(generator);
    float y = uniform_distribution_3_6(generator);
    safe_landing_planner.cloud_.push_back(pcl::PointXYZ(x, y, distribution(generator)));
  }

  for (int i = 0; i < 500; ++i) {
    float x = uniform_distribution_6_9(generator);
    float y = uniform_distribution_3_6(generator);
    safe_landing_planner.cloud_.push_back(pcl::PointXYZ(x, y, distribution(generator)));
  }

  std::normal_distribution<float> distribution_flat(0.0f, 0.1f);
  for (int i = 0; i < 500; ++i) {
    float x = uniform_distribution_3_6(generator);
    float y = uniform_distribution_3_6(generator);
    safe_landing_planner.cloud_.push_back(pcl::PointXYZ(x, y, distribution_flat(generator)));
  }
  safe_landing_planner.runSafeLandingPlanner();

  for (size_t i = 0; i < 10; i++) {
    for (size_t j = 0; j < 10; j++) {
      if (i >= 3 && i <= 5 && j >= 3 && j <= 5) {
        ASSERT_TRUE(safe_landing_planner.test_getGrid().land_(i, j)) << i << " " << j;
      } else {
        ASSERT_FALSE(safe_landing_planner.test_getGrid().land_(i, j)) << i << " " << j;
      }
    }
  }
}

TEST_F(SafeLandingPlannerTests, flat_bottom_half) {
  safe_landing_planner::SafeLandingPlannerNodeConfig config =
      safe_landing_planner::SafeLandingPlannerNodeConfig::__getDefault__();

  config.smoothing_size = -2;
  config.n_points_threshold = 4;
  config.min_n_land_cells = 4;
  config.cell_size = 1;

  safe_landing_planner.dynamicReconfigureSetParams(config, 1);

  // create pointcloud with 3x3m flat area on the right side
  std::default_random_engine generator(seed);
  std::normal_distribution<float> distribution(2.0f, 1.5f);

  for (int i = 0; i < 1000; ++i) {
    float min_x = -0.8f;
    float max_x = 4.2f;
    float x = min_x + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max_x - min_x)));
    float min_y = -1.1f;
    float max_y = 8.9f;
    float y = min_y + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max_y - min_y)));
    safe_landing_planner.cloud_.push_back(pcl::PointXYZ(x, y, distribution(generator)));
  }

  std::normal_distribution<float> distribution_flat(0.0f, 0.05f);
  for (int i = 0; i < 1000; ++i) {
    float min_x = 4.2f;
    float max_x = 9.2;
    float x = min_x + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max_x - min_x)));
    float min_y = -1.1f;
    float max_y = 8.9f;
    float y = min_y + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max_y - min_y)));
    safe_landing_planner.cloud_.push_back(pcl::PointXYZ(x, y, distribution_flat(generator)));
  }

  safe_landing_planner.runSafeLandingPlanner();

  for (size_t i = 0; i < 10; i++) {
    for (size_t j = 0; j < 5; j++) {
      ASSERT_FALSE(safe_landing_planner.test_getGrid().land_(j, i));
    }

    for (size_t j = 5; j < 10; j++) {
      ASSERT_TRUE(safe_landing_planner.test_getGrid().land_(j, i));
    }
  }
}

TEST_F(SafeLandingPlannerTests, binning_10_1) {
  safe_landing_planner::SafeLandingPlannerNodeConfig config =
      safe_landing_planner::SafeLandingPlannerNodeConfig::__getDefault__();
  config.alpha = 0.0;
  config.cell_size = 1;
  safe_landing_planner.dynamicReconfigureSetParams(config, 1);

  safe_landing_planner.cloud_.push_back(pcl::PointXYZ(-4.6f, -2.1f, 1.213f));
  safe_landing_planner.cloud_.push_back(pcl::PointXYZ(15.2f, -13.4f, 0.987f));

  safe_landing_planner.cloud_.push_back(pcl::PointXYZ(4.2f, 3.9f, 1.145f));   // index (5,5)
  safe_landing_planner.cloud_.push_back(pcl::PointXYZ(-0.5f, -1.f, 1.198f));  // index (0,0)
  safe_landing_planner.cloud_.push_back(pcl::PointXYZ(8.9f, 8.6f, 1.065f));   // index (9,9)

  safe_landing_planner.cloud_.push_back(pcl::PointXYZ(4.2f, 2.f, 1.287f));  // index (5, 3)

  safe_landing_planner.cloud_.push_back(pcl::PointXYZ(9.1f, 8.5f, 1.047f));  // index (9,9)
  safe_landing_planner.cloud_.push_back(pcl::PointXYZ(8.7f, 8.f, 1.134f));   // index (9,9)
  safe_landing_planner.cloud_.push_back(pcl::PointXYZ(8.7f, 8.f, 1.144f));   // index (9,9)
  safe_landing_planner.cloud_.push_back(pcl::PointXYZ(8.7f, 8.f, 1.104f));   // index (9,9)
  safe_landing_planner.cloud_.push_back(pcl::PointXYZ(8.7f, 8.f, 1.110f));   // index (9,9)

  safe_landing_planner.runSafeLandingPlanner();

  Eigen::Vector2i p = Eigen::Vector2i(0, 0);
  Eigen::Vector2i p1 = Eigen::Vector2i(9, 9);
  Eigen::Vector2i p2 = Eigen::Vector2i(5, 3);
  Eigen::Vector2i p3 = Eigen::Vector2i(5, 5);
  EXPECT_FLOAT_EQ(1.198f, safe_landing_planner.test_getGrid().getMean(p));
  EXPECT_FLOAT_EQ((1.065f + 1.047f + 1.134f + 1.144f + 1.104f + 1.110f) / 6.f,
                  safe_landing_planner.test_getGrid().getMean(p1));
  EXPECT_FLOAT_EQ(1.287f, safe_landing_planner.test_getGrid().getMean(p2));
  EXPECT_FLOAT_EQ(1.145f, safe_landing_planner.test_getGrid().getMean(p3));

  std::vector<float> v = {1.065f, 1.047f, 1.134f, 1.144f, 1.104f, 1.110f};
  float sum = std::accumulate(v.begin(), v.end(), 0.0);
  float mean = sum / v.size();
  float sq_sum = std::inner_product(v.begin(), v.end(), v.begin(), 0.0);
  float variance = (sq_sum / v.size() - mean * mean);
  ASSERT_NEAR(variance, safe_landing_planner.test_getGrid().getVariance(p1), 0.0001f);
}

TEST_F(SafeLandingPlannerTests, binning_12_4) {
  Eigen::Vector3f pos(-4.3f, 16.2f, 5.f);
  safe_landing_planner.setPose(pos, q);

  safe_landing_planner::SafeLandingPlannerNodeConfig config =
      safe_landing_planner::SafeLandingPlannerNodeConfig::__getDefault__();
  config.grid_size = 12.f;
  config.cell_size = 4.f;
  config.alpha = 0.0;
  safe_landing_planner.dynamicReconfigureSetParams(config, 1);

  Eigen::Vector2i p0 = Eigen::Vector2i(0, 0);
  safe_landing_planner.cloud_.push_back(pcl::PointXYZ(-10.289, 10.25, 0.023f));
  safe_landing_planner.cloud_.push_back(pcl::PointXYZ(-10.2, 11.2, 0.043f));
  safe_landing_planner.cloud_.push_back(pcl::PointXYZ(-6.33, 11.2, 0.086f));
  safe_landing_planner.cloud_.push_back(pcl::PointXYZ(-10.29, 11.2, 0.073f));

  Eigen::Vector2i p1 = Eigen::Vector2i(1, 0);
  safe_landing_planner.cloud_.push_back(pcl::PointXYZ(-6.01f, 12.3f, 0.036f));
  safe_landing_planner.cloud_.push_back(pcl::PointXYZ(-2.32f, 14.f, 0.045f));
  safe_landing_planner.cloud_.push_back(pcl::PointXYZ(-6.29f, 14.19f, 0.025f));
  safe_landing_planner.cloud_.push_back(pcl::PointXYZ(-4.59f, 10.21f, 0.085f));

  Eigen::Vector2i p2 = Eigen::Vector2i(2, 0);
  safe_landing_planner.cloud_.push_back(pcl::PointXYZ(1.69f, 12.3f, 0.036f));
  safe_landing_planner.cloud_.push_back(pcl::PointXYZ(-2.29f, 14.f, 1.045f));
  safe_landing_planner.cloud_.push_back(pcl::PointXYZ(-2.29f, 14.f, 0.333f));
  safe_landing_planner.cloud_.push_back(pcl::PointXYZ(-2.29f, 14.f, 1.995f));
  safe_landing_planner.cloud_.push_back(pcl::PointXYZ(-2.29f, 14.f, 0.245f));

  safe_landing_planner.runSafeLandingPlanner();

  std::vector<float> v0 = {0.023f, 0.043f, 0.086f, 0.073f};
  float sum = std::accumulate(v0.begin(), v0.end(), 0.0f);
  float mean = sum / v0.size();
  float sq_sum = std::inner_product(v0.begin(), v0.end(), v0.begin(), 0.0f);
  float variance = (sq_sum / v0.size() - mean * mean);

  EXPECT_FLOAT_EQ(mean, safe_landing_planner.test_getGrid().getMean(p0));
  EXPECT_FLOAT_EQ(variance, safe_landing_planner.test_getGrid().getVariance(p0));

  std::vector<float> v1 = {0.036f, 0.045f, 0.025f, 0.085f};
  sum = std::accumulate(v1.begin(), v1.end(), 0.0f);
  mean = sum / v1.size();
  sq_sum = std::inner_product(v1.begin(), v1.end(), v1.begin(), 0.0f);
  variance = (sq_sum / v1.size() - mean * mean);
  EXPECT_FLOAT_EQ(mean, safe_landing_planner.test_getGrid().getMean(p1));
  EXPECT_FLOAT_EQ(variance, safe_landing_planner.test_getGrid().getVariance(p1));

  std::vector<float> v2 = {0.036f, 1.045f, 0.333f, 1.995f, 0.245f};
  sum = std::accumulate(v2.begin(), v2.end(), 0.0f);
  mean = sum / v2.size();
  sq_sum = std::inner_product(v2.begin(), v2.end(), v2.begin(), 0.0f);
  variance = (sq_sum / v2.size() - mean * mean);
  EXPECT_FLOAT_EQ(mean, safe_landing_planner.test_getGrid().getMean(p2));
  EXPECT_FLOAT_EQ(variance, safe_landing_planner.test_getGrid().getVariance(p2));
}

TEST_F(SafeLandingPlannerTests, mean) {
  Eigen::Vector3f pos(5.f, 5.f, 5.f);
  safe_landing_planner.setPose(pos, q);

  safe_landing_planner::SafeLandingPlannerNodeConfig config =
      safe_landing_planner::SafeLandingPlannerNodeConfig::__getDefault__();

  config.smoothing_size = 1;
  config.n_points_threshold = 1;
  config.min_n_land_cells = 1;
  config.cell_size = 1;
  config.max_n_mean_diff_cells = 1;

  safe_landing_planner.dynamicReconfigureSetParams(config, 1);

  std::default_random_engine generator(seed);
  std::normal_distribution<float> distribution_flat_ground(0.0f, 0.05f);
  for (int i = 0; i < 2000; ++i) {
    float min = 0.f;
    float max = 10.f;
    float x = min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
    float y = min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
    safe_landing_planner.cloud_.push_back(pcl::PointXYZ(x, y, distribution_flat_ground(generator)));
  }

  std::normal_distribution<float> distribution_flat_box(1.5f, 0.05f);
  for (int i = 0; i < 500; ++i) {
    float min = 4.f;
    float max = 5.f;
    float x = min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
    float y = min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
    safe_landing_planner.cloud_.push_back(pcl::PointXYZ(x, y, distribution_flat_box(generator)));
  }

  safe_landing_planner.runSafeLandingPlanner();

  for (int i = 0; i < safe_landing_planner.test_getGrid().land_.rows(); i++) {
    for (int j = 0; j < safe_landing_planner.test_getGrid().land_.cols(); j++) {
      if (i == 4 && j == 4) {
        ASSERT_FALSE(safe_landing_planner.test_getGrid().land_(j, i));
      } else {
        ASSERT_TRUE(safe_landing_planner.test_getGrid().land_(j, i));
      }
    }
  }
}

TEST_F(SafeLandingPlannerTests, test_mean_variance) {
  Eigen::Vector3f pos(-4.3f, 16.2f, 5.f);
  safe_landing_planner.setPose(pos, q);

  safe_landing_planner::SafeLandingPlannerNodeConfig config =
      safe_landing_planner::SafeLandingPlannerNodeConfig::__getDefault__();
  config.grid_size = 12.f;
  config.cell_size = 4.f;
  config.alpha = 0.0;
  safe_landing_planner.dynamicReconfigureSetParams(config, 1);
  std::vector<float> v0;
  Eigen::Vector2i p0 = Eigen::Vector2i(0, 0);
  for (int i = 0; i < 1000; ++i) {
    float min = 4.f;
    float max = 10.f;
    float z = min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
    v0.push_back(z);
    safe_landing_planner.cloud_.push_back(pcl::PointXYZ(-10.289, 10.25, z));
  }

  safe_landing_planner.runSafeLandingPlanner();

  float sum = std::accumulate(v0.begin(), v0.end(), 0.0f);
  float mean = sum / v0.size();
  float sq_sum = std::inner_product(v0.begin(), v0.end(), v0.begin(), 0.0f);
  float variance = (sq_sum / v0.size() - mean * mean);

  ASSERT_NEAR(mean, safe_landing_planner.test_getGrid().getMean(p0), 0.001f);
  ASSERT_NEAR(variance, safe_landing_planner.test_getGrid().getVariance(p0), 0.001f);
}

TEST_F(SafeLandingPlannerTests, test_combine) {
  safe_landing_planner::SafeLandingPlannerNodeConfig config =
      safe_landing_planner::SafeLandingPlannerNodeConfig::__getDefault__();
  config.grid_size = 3;
  config.cell_size = 1;
  config.alpha = 0.9;

  safe_landing_planner.dynamicReconfigureSetParams(config, 1);
  Eigen::Vector2i x = Eigen::Vector2i(0, 0);
  safe_landing_planner.test_getGrid().setMean(x, 1.f);
  safe_landing_planner.test_getPrevGrid().setMean(x, 1.5f);

  Eigen::Vector2i y = Eigen::Vector2i(0, 1);
  safe_landing_planner.test_getGrid().setMean(y, 3.21f);
  safe_landing_planner.test_getPrevGrid().setMean(y, 5.15f);

  Eigen::Vector2i z = Eigen::Vector2i(0, 2);
  safe_landing_planner.test_getGrid().setMean(z, 1.75f);
  safe_landing_planner.test_getPrevGrid().setMean(z, 1.89f);

  safe_landing_planner.test_getGrid().combine(safe_landing_planner.test_getPrevGrid(), 0.9);
  ASSERT_NEAR(0.9f * 1.5f + 0.1f * 1.f, safe_landing_planner.test_getGrid().getMean(x), 0.001f);
  ASSERT_NEAR(0.9f * 5.15f + 0.1f * 3.21f, safe_landing_planner.test_getGrid().getMean(y), 0.001f);
  ASSERT_NEAR(0.9f * 1.89f + 0.1f * 1.75f, safe_landing_planner.test_getGrid().getMean(z), 0.001f);
}
