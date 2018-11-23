#include <gtest/gtest.h>

#include "../src/nodes/local_planner.h"

// Stateless tests:
// Create some hardcoded scan data of obstacles in different positions
// For each one check that the planner response is correct

class LocalPlannerTests : public ::testing::Test {
 public:
  LocalPlanner planner;

  void SetUp() override {
    ros::Time::init();
    geometry_msgs::PoseStamped msg;
    msg.header.seq = 42;
    msg.header.stamp = ros::Time(500, 0);
    msg.header.frame_id = 1;

    // start with basic pose
    msg.pose.position.x = 0;
    msg.pose.position.y = 0;
    msg.pose.position.z = 0;
    msg.pose.orientation.w = 1;

    // goal straight in front, 100m away
    planner.goal_x_param_ = 100;
    planner.goal_y_param_ = 0;
    planner.goal_z_param_ = 0;
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

  // THEN: it shouldn't modify the path
  avoidanceOutput avoidance;
  planner.getAvoidanceOutput(avoidance);
  EXPECT_FALSE(avoidance.obstacle_ahead);
}

TEST_F(LocalPlannerTests, all_obstacles) {
  // GIVEN: a local planner, a scan with obstacles everywhere, pose and goal
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (float y = -3; y < 3; y += 0.1) {
    for (float z = -2; z < 2; z += 0.1) {
      cloud.push_back(pcl::PointXYZ(1, y, z));
    }
  }

  planner.complete_cloud_.push_back(std::move(cloud));

  // WHEN: we run the local planner
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
