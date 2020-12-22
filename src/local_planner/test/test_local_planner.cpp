#include <gtest/gtest.h>

#include <cmath>

#include "../include/local_planner/local_planner.h"
#include "avoidance/common.h"

#define TO_DEG 180.f / M_PI_F
#define TO_RAD M_PI_F / 180.f

// Stateless tests:
// Create some hardcoded scan data of obstacles in different positions
// For each one check that the planner response is correct
using namespace avoidance;

class LocalPlannerTests : public ::testing::Test {
 public:
  LocalPlanner planner;

  void SetUp() override {
    ros::Time::init();

    planner.setDefaultPx4Parameters();
    avoidance::LocalPlannerNodeConfig config = avoidance::LocalPlannerNodeConfig::__getDefault__();
    planner.dynamicReconfigureSetParams(config, 1);
    planner.setFOV(0, FOV(0.0f, 0.0f, 59.0f, 46.0f));

    // start with basic pose
    Eigen::Vector3f pos(0.f, 0.f, 0.f);
    Eigen::Vector3f vel(0.f, 0.f, 0.f);
    Eigen::Quaternionf q(1.f, 0.f, 0.f, 0.f);
    planner.currently_armed_ = false;
    planner.setState(pos, vel, q);

    // rise to altitude
    planner.currently_armed_ = true;
    pos.z() = 30.f;
    planner.setState(pos, vel, q);

    // goal straight in front, 100m away
    Eigen::Vector3f goal(100.f, 0.f, 30.f);
    planner.setGoal(goal);

    planner.original_cloud_vector_.clear();
  }
  void TearDown() override {}
};

TEST_F(LocalPlannerTests, no_obstacles) {
  // GIVEN: a local planner, a scan with no obstacles, pose and goal
  planner.original_cloud_vector_.emplace_back();

  // WHEN: we run the local planner
  planner.runPlanner();
  planner.runPlanner();

  // THEN: it shouldn't find any obstacles
  avoidanceOutput output = planner.getAvoidanceOutput();

  // AND: the scan should contain NANs outside the FOV and out of range data inside
  sensor_msgs::LaserScan scan;
  planner.getObstacleDistanceData(scan);
  const std::vector<FOV> fov_vec = planner.getFOV();
  Eigen::Vector3f position = planner.getPosition();
  float curr_yaw_deg = planner.getOrientation();

  for (size_t i = 0; i < scan.ranges.size(); i++) {
    int hist_idx = (i + GRID_LENGTH_Z / 2) % GRID_LENGTH_Z;
    if (histogramIndexYawInsideFOV(fov_vec, hist_idx, position, curr_yaw_deg)) {
      EXPECT_GT(scan.ranges[i], scan.range_max);
    } else {
      EXPECT_TRUE(std::isnan(scan.ranges[i]));  // outside the FOV the values should be NAN
    }
  }
}

TEST_F(LocalPlannerTests, all_obstacles) {
  // GIVEN: a local planner, a scan with obstacles everywhere, pose and goal
  float shift = 0.f;
  float distance = 2.f;
  float fov_half_y = distance * std::tan(planner.getHFOV(0) * TO_RAD / 2.f);
  float max_y = shift + fov_half_y, min_y = shift - fov_half_y;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (float y = min_y; y <= max_y; y += 0.01f) {
    for (float z = -1.f; z <= 1.f; z += 0.1f) {
      cloud.push_back(pcl::PointXYZ(distance, y, z + 30.f));
    }
  }
  planner.original_cloud_vector_.push_back(std::move(cloud));

  // WHEN: we run the local planner
  planner.runPlanner();

  // THEN: it should get a scan showing the obstacle
  sensor_msgs::LaserScan scan;
  planner.getObstacleDistanceData(scan);
  const std::vector<FOV> fov_vec = planner.getFOV();
  Eigen::Vector3f position = planner.getPosition();
  float curr_yaw_deg = planner.getOrientation();
  int idx_lower_obstacle_boundary_bin =
      static_cast<int>((M_PI_F / 2 - atan2(max_y, distance)) / (scan.angle_increment));
  int idx_upper_obstacle_boundary_bin =
      static_cast<int>((M_PI_F / 2 - atan2(min_y, distance)) / (scan.angle_increment));

  for (size_t i = 0; i < scan.ranges.size(); i++) {
    int hist_idx = (i + GRID_LENGTH_Z / 2) % GRID_LENGTH_Z;
    if (!histogramIndexYawInsideFOV(fov_vec, hist_idx, position, curr_yaw_deg)) {
      EXPECT_TRUE(std::isnan(scan.ranges[i]));  // outside the FOV the values should be NAN
    } else if (idx_lower_obstacle_boundary_bin <= i && i <= idx_upper_obstacle_boundary_bin) {
      EXPECT_LT(scan.ranges[i], distance * 1.5f);
    } else {
      EXPECT_GT(scan.ranges[i], scan.range_max);
    }
  }

  // WHEN: we run the local planner again
  planner.runPlanner();

  // THEN: it should detect the obstacle and go left
  avoidanceOutput output = planner.getAvoidanceOutput();

  ASSERT_GE(output.path_node_positions.size(), 2);
  float node_max_y = 0.f;
  float node_min_y = 0.f;
  for (auto it = output.path_node_positions.rbegin(); it != output.path_node_positions.rend(); ++it) {
    auto node = *it;
    if (node.x() > distance) break;
    if (node.y() > node_max_y) node_max_y = node.y();
    if (node.y() < node_min_y) node_min_y = node.y();
  }

  bool steer_clear = node_max_y > max_y || node_min_y < min_y;
  EXPECT_TRUE(steer_clear);
}

TEST_F(LocalPlannerTests, obstacles_right) {
  // GIVEN: a local planner, a scan with obstacles on the right, pose and goal
  float shift = -0.5f;
  float distance = 2.f;
  float fov_half_y = distance * std::tan(planner.getHFOV(0) * TO_RAD / 2.f);
  float max_y = shift + fov_half_y, min_y = shift - fov_half_y;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (float y = min_y; y <= max_y; y += 0.01f) {
    for (float z = -1.f; z <= 1.f; z += 0.1f) {
      cloud.push_back(pcl::PointXYZ(distance, y, z + 30));
    }
  }
  planner.original_cloud_vector_.push_back(std::move(cloud));

  // WHEN: we run the local planner
  planner.runPlanner();

  // THEN: it should get a scan showing the obstacle
  sensor_msgs::LaserScan scan;
  planner.getObstacleDistanceData(scan);
  const std::vector<FOV> fov_vec = planner.getFOV();
  Eigen::Vector3f position = planner.getPosition();
  float curr_yaw_deg = planner.getOrientation();
  int idx_lower_obstacle_boundary_bin =
      static_cast<int>((M_PI_F / 2 - atan2(max_y, distance)) / (scan.angle_increment));
  int idx_upper_obstacle_boundary_bin =
      static_cast<int>((M_PI_F / 2 - atan2(min_y, distance)) / (scan.angle_increment));

  for (size_t i = 0; i < scan.ranges.size(); i++) {
    int hist_idx = (i + GRID_LENGTH_Z / 2) % GRID_LENGTH_Z;
    if (!histogramIndexYawInsideFOV(fov_vec, hist_idx, position, curr_yaw_deg)) {
      EXPECT_TRUE(std::isnan(scan.ranges[i]));  // outside the FOV the values should be NAN
    } else if (idx_lower_obstacle_boundary_bin <= i && i <= idx_upper_obstacle_boundary_bin) {
      EXPECT_LT(scan.ranges[i], distance * 1.5f);
    } else {
      EXPECT_GT(scan.ranges[i], scan.range_max);
    }
  }

  // WHEN: we run the local planner again
  planner.runPlanner();

  // THEN: it should modify the path to the left
  avoidanceOutput output = planner.getAvoidanceOutput();

  ASSERT_GE(output.path_node_positions.size(), 2);
  float node_max_y = 0.f;
  for (auto it = output.path_node_positions.rbegin(); it != output.path_node_positions.rend(); ++it) {
    auto node = *it;
    if (node.x() > distance) break;
    if (node.y() > node_max_y) node_max_y = node.y();
  }
  EXPECT_GT(node_max_y, max_y);
}

TEST_F(LocalPlannerTests, obstacles_left) {
  // GIVEN: a local planner, a scan with obstacles on the left, pose and goal
  float shift = 0.5f;
  float distance = 2.f;
  float fov_half_y = distance * std::tan(planner.getHFOV(0) * TO_RAD / 2.f);
  float max_y = shift + fov_half_y, min_y = shift - fov_half_y;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (float y = min_y; y <= max_y; y += 0.01f) {
    for (float z = -1.f; z <= 1.f; z += 0.1f) {
      cloud.push_back(pcl::PointXYZ(distance, y, z + 30.f));
    }
  }
  planner.original_cloud_vector_.push_back(std::move(cloud));

  // WHEN: we run the local planner
  planner.runPlanner();

  // THEN: it should get a scan showing the obstacle
  sensor_msgs::LaserScan scan;
  planner.getObstacleDistanceData(scan);
  const std::vector<FOV> fov_vec = planner.getFOV();
  Eigen::Vector3f position = planner.getPosition();
  float curr_yaw_deg = planner.getOrientation();
  int idx_lower_obstacle_boundary_bin =
      static_cast<int>((M_PI_F / 2 - atan2(max_y, distance)) / (scan.angle_increment));
  int idx_upper_obstacle_boundary_bin =
      static_cast<int>((M_PI_F / 2 - atan2(min_y, distance)) / (scan.angle_increment));

  for (size_t i = 0; i < scan.ranges.size(); i++) {
    int hist_idx = (i + GRID_LENGTH_Z / 2) % GRID_LENGTH_Z;
    if (!histogramIndexYawInsideFOV(fov_vec, hist_idx, position, curr_yaw_deg)) {
      EXPECT_TRUE(std::isnan(scan.ranges[i]));  // outside the FOV the values should be NAN
    } else if (idx_lower_obstacle_boundary_bin <= i && i <= idx_upper_obstacle_boundary_bin) {
      EXPECT_LT(scan.ranges[i], distance * 1.5f);
    } else {
      EXPECT_GT(scan.ranges[i], scan.range_max);
    }
  }

  // WHEN: we run the local planner again
  planner.runPlanner();

  // THEN: it should modify the path to the right
  avoidanceOutput output = planner.getAvoidanceOutput();

  ASSERT_GE(output.path_node_positions.size(), 2);
  float node_min_y = 0.f;
  for (auto it = output.path_node_positions.rbegin(); it != output.path_node_positions.rend(); ++it) {
    auto node = *it;
    if (node.x() > distance) break;
    if (node.y() < node_min_y) node_min_y = node.y();
  }
  EXPECT_LT(node_min_y, min_y);
}
