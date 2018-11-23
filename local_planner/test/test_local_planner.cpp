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
  }
  void TearDown() override {}
};

TEST_F(LocalPlannerTests, no_obstacles) {
  // GIVEN: a local planner, a scan with no obstacles, pose and goal
  planner.complete_cloud_.clear();
  planner.complete_cloud_.emplace_back();

  // WHEN: we run the local planner
  planner.runPlanner();

  // THEN: it shouldn't modify the path
  avoidanceOutput avoidance;
  planner.getAvoidanceOutput(avoidance);
  EXPECT_FALSE(avoidance.obstacle_ahead);
}


