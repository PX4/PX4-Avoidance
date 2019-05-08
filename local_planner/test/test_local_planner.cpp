#include <gtest/gtest.h>

#include <cmath>

#include "../include/local_planner/local_planner.h"
#include "avoidance/common.h"

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
    avoidance::LocalPlannerNodeConfig config =
        avoidance::LocalPlannerNodeConfig::__getDefault__();
    planner.dynamicReconfigureSetParams(config, 1);

    // start with basic pose
    Eigen::Vector3f pos(0.f, 0.f, 0.f);
    Eigen::Quaternionf q(1.f, 0.f, 0.f, 0.f);
    planner.currently_armed_ = false;
    planner.setPose(pos, q);

    // rise to altitude
    planner.currently_armed_ = true;
    pos.z() = 30.f;
    planner.setPose(pos, q);

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
  EXPECT_FALSE(output.obstacle_ahead);

  // AND: the scan shouldn't have any data
  sensor_msgs::LaserScan scan;
  planner.getObstacleDistanceData(scan);

  for (size_t i = 0; i < scan.ranges.size(); i++) {
    EXPECT_GT(scan.ranges[i], scan.range_max);
  }
}

TEST_F(LocalPlannerTests, all_obstacles) {
  // GIVEN: a local planner, a scan with obstacles everywhere, pose and goal
  float shift = 0.f;
  float distance = 2.f;
  float fov_half_y =
      distance * std::tan(planner.h_FOV_deg_ * M_PI_F / 180.f / 2.f);
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

  for (size_t i = 0; i < scan.ranges.size(); i++) {
    if (10 <= i && i <= 19)
      EXPECT_LT(scan.ranges[i], distance * 1.5f);
    else
      EXPECT_GT(scan.ranges[i], scan.range_max);
  }

  // WHEN: we run the local planner again
  planner.runPlanner();

  // THEN: it should detect the obstacle and go left
  avoidanceOutput output = planner.getAvoidanceOutput();

  EXPECT_TRUE(output.obstacle_ahead);
  ASSERT_GE(output.path_node_positions.size(), 2);
  float node_max_y = 0.f;
  float node_min_y = 0.f;
  for (auto it = output.path_node_positions.rbegin();
       it != output.path_node_positions.rend(); ++it) {
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
  float fov_half_y =
      distance * std::tan(planner.h_FOV_deg_ * M_PI_F / 180.f / 2.f);
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

  for (size_t i = 0; i < scan.ranges.size(); i++) {
    if (12 <= i && i <= 19)
      EXPECT_LT(scan.ranges[i], distance * 1.5f);
    else
      EXPECT_GT(scan.ranges[i], scan.range_max);
  }

  // WHEN: we run the local planner again
  planner.runPlanner();

  // THEN: it should modify the path to the left
  avoidanceOutput output = planner.getAvoidanceOutput();

  EXPECT_TRUE(output.obstacle_ahead);
  ASSERT_GE(output.path_node_positions.size(), 2);
  float node_max_y = 0.f;
  for (auto it = output.path_node_positions.rbegin();
       it != output.path_node_positions.rend(); ++it) {
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
  float fov_half_y =
      distance * std::tan(planner.h_FOV_deg_ * M_PI_F / 180.f / 2.f);
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
  for (size_t i = 0; i < scan.ranges.size(); i++) {
    if (10 <= i && i <= 17)
      EXPECT_LT(scan.ranges[i], distance * 1.5f);
    else
      EXPECT_GT(scan.ranges[i], scan.range_max);
  }

  // WHEN: we run the local planner again
  planner.runPlanner();

  // THEN: it should modify the path to the right
  avoidanceOutput output = planner.getAvoidanceOutput();

  EXPECT_TRUE(output.obstacle_ahead);
  ASSERT_GE(output.path_node_positions.size(), 2);
  float node_min_y = 0.f;
  for (auto it = output.path_node_positions.rbegin();
       it != output.path_node_positions.rend(); ++it) {
    auto node = *it;
    if (node.x() > distance) break;
    if (node.y() < node_min_y) node_min_y = node.y();
  }
  EXPECT_LT(node_min_y, min_y);
}
