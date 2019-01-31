#include <gtest/gtest.h>

#include "../src/nodes/common.h"
#include "../src/nodes/star_planner.h"
#include "../src/nodes/tree_node.h"

using namespace avoidance;

class StarPlannerTests : public ::testing::Test {
 public:
  StarPlanner star_planner;
  float obstacle_half_height = 0.5f;
  float obstacle_min_x = 0.0f;
  float obstacle_max_x = 2.5f;
  float obstacle_y = 2.0f;
  geometry_msgs::Point goal;
  geometry_msgs::PoseStamped position;

  void SetUp() override {
    ros::Time::init();

    avoidance::LocalPlannerNodeConfig config =
        avoidance::LocalPlannerNodeConfig::__getDefault__();
    config.childs_per_node_ = 2;
    config.n_expanded_nodes_ = 10;
    star_planner.dynamicReconfigureSetStarParams(config, 1);

    position.pose.position.x = 1.2;
    position.pose.position.y = 0.4;
    position.pose.position.z = 4.0;
    position.pose.orientation.x = 0.0;
    position.pose.orientation.y = 0.0;
    position.pose.orientation.z = 0.0;
    position.pose.orientation.w = 0.0;

    goal.x = 2.0;
    goal.y = 14.0;
    goal.z = 4.0;

    pcl::PointCloud<pcl::PointXYZ> cloud;
    for (float x = obstacle_min_x; x < obstacle_max_x; x += 0.05f) {
      for (float z = goal.z - obstacle_half_height;
           z < goal.z + obstacle_half_height; z += 0.05f) {
        cloud.push_back(pcl::PointXYZ(x, obstacle_y, z));
      }
    }
    costParameters cost_params;
    const pcl::PointCloud<pcl::PointXYZ> reprojected_points;
    const std::vector<int> reprojected_points_age;

    star_planner.setParams(cost_params);
    star_planner.setFOV(270.0f, 45.0f);
    star_planner.setReprojectedPoints(reprojected_points,
                                      reprojected_points_age);
    star_planner.setPose(position, 0.0f);
    star_planner.setGoal(goal);
    star_planner.setCloud(cloud);
  }
  void TearDown() override {}
};

TEST_F(StarPlannerTests, buildTree) {
  // GIVEN: a vehicle position, a goal, and an obstacle in between the straight
  // line position-goal

  float dist_to_goal = 1000.0f;

  // WHEN: we build the tree for 15 times
  for (size_t i = 0; i < 15; i++) {
    star_planner.buildLookAheadTree();
    for (auto node : star_planner.tree_) {
      // THEN: we expect each tree node position not to be close to the obstacle
      Eigen::Vector3f n = node.getPosition();
      bool node_inside_obstacle =
          n.x() > obstacle_min_x && n.x() < obstacle_max_x &&
          n.y() > obstacle_y - 0.1f && n.y() < obstacle_y + 0.1 &&
          n.z() > 4.0f - obstacle_half_height &&
          n.z() < 4.0f + obstacle_half_height;
      EXPECT_FALSE(node_inside_obstacle);
    }

    // THEN: we expect the distance between the vehicle and the goal to shorten
    // at each iteration
    float tmp_dist_to_goal =
        (toEigen(goal) - toEigen(position.pose.position)).norm();
    ASSERT_LT(tmp_dist_to_goal, dist_to_goal);
    dist_to_goal = tmp_dist_to_goal;

    // we set the vehicle position to be the first node position after the
    // origin for the next algorithm iterarion
    position.pose.position = toPoint(star_planner.tree_[1].getPosition());
    star_planner.setPose(position, 0.0);
  }
}
