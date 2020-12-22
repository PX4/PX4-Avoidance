#include <gtest/gtest.h>

#include "../include/local_planner/star_planner.h"
#include "../include/local_planner/tree_node.h"
#include "avoidance/common.h"

using namespace avoidance;

class StarPlannerBasicTests : public ::testing::Test, public StarPlanner {
  void SetUp() override{};
  void TearDown() override{};
};

class StarPlannerTests : public ::testing::Test {
 public:
  StarPlanner star_planner;
  float obstacle_half_height = 0.5f;
  float obstacle_min_x = 0.0f;
  float obstacle_max_x = 2.5f;
  float obstacle_y = 2.0f;
  Eigen::Vector3f goal;
  Eigen::Vector3f position;
  Eigen::Vector3f velocity;

  void SetUp() override {
    ros::Time::init();

    avoidance::LocalPlannerNodeConfig config = avoidance::LocalPlannerNodeConfig::__getDefault__();
    config.children_per_node_ = 2;
    config.n_expanded_nodes_ = 10;
    config.tree_node_distance_ = 1.0;
    star_planner.dynamicReconfigureSetStarParams(config, 1);

    position.x() = 1.2f;
    position.y() = 0.4f;
    position.z() = 4.0f;

    velocity.x() = 0.0f;
    velocity.y() = 0.0f;
    velocity.z() = 0.0f;

    goal.x() = 2.0f;
    goal.y() = 14.0f;
    goal.z() = 4.0f;

    pcl::PointCloud<pcl::PointXYZI> cloud;
    for (float x = obstacle_min_x; x < obstacle_max_x; x += 0.05f) {
      for (float z = goal.z() - obstacle_half_height; z < goal.z() + obstacle_half_height; z += 0.05f) {
        cloud.push_back(toXYZI(x, obstacle_y, z, 0));
      }
    }
    costParameters cost_params;

    star_planner.setParams(cost_params);
    star_planner.setPointcloud(cloud);
    star_planner.setPose(position, velocity);
    star_planner.setGoal(goal);
    star_planner.setClosestPointOnLine(goal);
  }
  void TearDown() override {}
};

TEST_F(StarPlannerTests, buildTree) {
  // GIVEN: a vehicle position, a goal, and an obstacle in between the straight
  // line position-goal

  float dist_to_goal = 1000.0f;
  float min_dist_to_goal = dist_to_goal;
  // WHEN: we build the tree for 15 times
  for (size_t i = 0; i < 15; i++) {
    star_planner.buildLookAheadTree();
    for (auto node : star_planner.tree_) {
      // THEN: we expect each tree node position not to be close to the obstacle
      Eigen::Vector3f n = node.getPosition();
      bool node_inside_obstacle = n.x() > obstacle_min_x && n.x() < obstacle_max_x && n.y() > obstacle_y - 0.1f &&
                                  n.y() < obstacle_y + 0.1f && n.z() > 4.0f - obstacle_half_height &&
                                  n.z() < 4.0f + obstacle_half_height;
      EXPECT_FALSE(node_inside_obstacle);
    }

    // THEN: we expect the distance between the vehicle and the goal to shorten
    // at each iteration
    float tmp_dist_to_goal = (goal - position).norm();
    EXPECT_LT(tmp_dist_to_goal, min_dist_to_goal * 1.1);
    dist_to_goal = tmp_dist_to_goal;
    if (dist_to_goal < min_dist_to_goal) min_dist_to_goal = dist_to_goal;

    // we set the vehicle position to be the first node position after the
    // origin for the next algorithm iterarion
    position = star_planner.tree_[1].getPosition();
    star_planner.setPose(position, velocity);
  }
}

TEST_F(StarPlannerTests, heuristicFunction) {}
