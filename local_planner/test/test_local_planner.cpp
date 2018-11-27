#include <gtest/gtest.h>

#include <cmath>

#include "../src/nodes/local_planner.h"

// Stateless tests:
// Create some hardcoded scan data of obstacles in different positions
// For each one check that the planner response is correct

class LocalPlannerTests : public ::testing::Test {
 public:
  LocalPlanner planner;

  void SetUp() override {
    ros::Time::init();

    avoidance::LocalPlannerNodeConfig config =
        avoidance::LocalPlannerNodeConfig::__getDefault__();
    config.send_obstacles_fcu_ = true;
    planner.dynamicReconfigureSetParams(config, 1);

    // start with basic pose
    geometry_msgs::PoseStamped msg;
    msg.header.seq = 42;
    msg.header.stamp = ros::Time(500, 0);
    msg.header.frame_id = 1;
    msg.pose.position.x = 0;
    msg.pose.position.y = 0;
    msg.pose.position.z = 0;
    msg.pose.orientation.w = 1;
    planner.currently_armed_ = false;
    planner.setPose(msg);

    // rise to altitude
    planner.currently_armed_ = true;
    msg.pose.position.z = 30;
    planner.setPose(msg);

    // goal straight in front, 100m away
    planner.goal_x_param_ = 100;
    planner.goal_y_param_ = 0;
    planner.goal_z_param_ = 30;
    planner.setGoal();

    planner.complete_cloud_.clear();
  }
  void TearDown() override {}
};

TEST_F(LocalPlannerTests, no_obstacles) {
  // GIVEN: a local planner, a scan with no obstacles, pose and goal
  planner.complete_cloud_.emplace_back();

  // WHEN: we run the local planner
  planner.runPlanner();
  planner.runPlanner();

  // THEN: it shouldn't find any obstacles
  avoidanceOutput avoidance;
  planner.getAvoidanceOutput(avoidance);
  EXPECT_FALSE(avoidance.obstacle_ahead);

  // AND: the scan shouldn't have any data
  sensor_msgs::LaserScan scan;
  planner.sendObstacleDistanceDataToFcu(scan);

  for (size_t i = 0; i < scan.ranges.size(); i++) {
    EXPECT_GT(scan.ranges[i], scan.range_max);
  }
}

TEST_F(LocalPlannerTests, all_obstacles) {
  // GIVEN: a local planner, a scan with obstacles everywhere, pose and goal
  pcl::PointCloud<pcl::PointXYZ> cloud;
  float max_y = std::tan(planner.h_FOV_ * M_PI / 180.f / 2.f), min_y = -max_y;
  for (float y = min_y; y <= max_y; y += 0.01) {
    for (float z = -1; z <= 1; z += 0.1) {
      cloud.push_back(pcl::PointXYZ(1, y, z + 30));
    }
  }

  planner.complete_cloud_.push_back(std::move(cloud));

  // WHEN: we run the local planner
  planner.runPlanner();

  // THEN: it should get a scan showing the obstacle
  sensor_msgs::LaserScan scan;
  planner.sendObstacleDistanceDataToFcu(scan);

  for (size_t i = 0; i < scan.ranges.size(); i++) {
    if (10 <= i && i <= 18)
    // width determined empirically, TODO investigate why it isn't symmetrical
    {
      EXPECT_LT(scan.ranges[i], 2.f);
    } else {
      EXPECT_GT(scan.ranges[i], scan.range_max);
    }
  }

  // WHEN: we run the local planner again
  planner.runPlanner();

  // THEN: it should stop the drone
  avoidanceOutput avoidance;
  planner.getAvoidanceOutput(avoidance);

  EXPECT_TRUE(avoidance.obstacle_ahead);
}

/*
TEST_F(LocalPlannerTests, obstacles_left) {
  // GIVEN: a local planner, a scan with obstacles on the left, pose and goal

  // WHEN: we run the local planner

  // THEN: it should modify the path to the right
}

TEST_F(LocalPlannerTests, obstacles_right) {
  // GIVEN: a local planner, a scan with obstacles on the right, pose and goal

  // WHEN: we run the local planner

  // THEN: it should modify the path to the left
}

TEST_F(LocalPlannerTests, obstacles_middle) {
  // GIVEN: a local planner, a scan with obstacles in the middle, pose and goal

  // WHEN: we run the local planner

  // THEN: it should modify the path to the right or left or stop, but not
  // straight
}*/

// Stateful tests (local planning with history) - should these be done as a full
// simulation?
// Set up a more complex 3D environment
// Put the camera in a location
// Set a goal
// Check that with incremental movements it doesn't get stuck in local minima
