#include <gtest/gtest.h>

#include "../include/local_planner/common.h"
#include "../include/local_planner/star_planner.h"
#include "../include/local_planner/tree_node.h"

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
  geometry_msgs::Point goal;
  Eigen::Vector3f position;


  void SetUp() override {
    ros::Time::init();

    avoidance::LocalPlannerNodeConfig config =
        avoidance::LocalPlannerNodeConfig::__getDefault__();
    config.children_per_node_ = 2;
    config.n_expanded_nodes_ = 10;
    star_planner.dynamicReconfigureSetStarParams(config, 1);

    position.x() = 1.2f;
    position.y() = 0.4f;
    position.z() = 4.0f;

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
        (toEigen(goal) - position).norm();
    ASSERT_LT(tmp_dist_to_goal, dist_to_goal);
    dist_to_goal = tmp_dist_to_goal;

    // we set the vehicle position to be the first node position after the
    // origin for the next algorithm iterarion
    position = star_planner.tree_[1].getPosition();
    star_planner.setPose(position, 0.0);
  }
}

TEST_F(StarPlannerBasicTests, treeCostFunctionTargetCost) {
  // GIVEN: a tree, the last path and two different goal locations
  Eigen::Vector3f goal1(5.f, 1.f, 0.f);
  Eigen::Vector3f goal2(5.f, 3.f, 0.f);

  // insert tree root
  Eigen::Vector3f tree_root(0.f, 0.f, 0.f);
  tree_.push_back(TreeNode(0, 0, tree_root));
  tree_.back().yaw_ = 90.0;  // drone looks straight ahead
  tree_.back().last_z_ = tree_.back().yaw_;

  // insert first Node
  Eigen::Vector3f node1(1.f, 0.f, 0.f);
  tree_.push_back(TreeNode(0, 1, node1));
  tree_.back().last_e_ = 0.f;
  tree_.back().last_z_ = 90.f;

  // last path equal to the given nodes
  std::vector<geometry_msgs::Point> path_node_positions_;
  path_node_positions_.push_back(toPoint(node1));
  path_node_positions_.push_back(toPoint(tree_root));

  // WHEN: we calculate the cost of node 1 for two different goal locations
  setGoal(toPoint(goal1));
  tree_age_ = 1;
  double cost1 = treeCostFunction(1);
  setGoal(toPoint(goal2));
  tree_age_ = 1;
  double cost2 = treeCostFunction(1);

  // THEN: The cost1 should be less than cost2, as in case 1 the node heads
  // closer to the goal
  EXPECT_GT(cost2, cost1);
}

TEST_F(StarPlannerBasicTests, treeCostFunctionOldPathCost) {
  // GIVEN: a tree, a goal and the last path
  StarPlanner star_planner;
  Eigen::Vector3f goal(5.f, 0.f, 0.f);
  setGoal(toPoint(goal));
  tree_age_ = 1;

  // insert tree root
  Eigen::Vector3f tree_root(0.f, 0.f, 0.f);
  tree_.push_back(TreeNode(0, 0, tree_root));
  tree_.back().yaw_ = 90.0;  // drone looks straight ahead
  tree_.back().last_z_ = tree_.back().yaw_;

  // insert first Node
  Eigen::Vector3f node1(1.f, 0.f, 0.f);
  tree_.push_back(TreeNode(0, 1, node1));
  tree_.back().last_e_ = 0.f;
  tree_.back().last_z_ = 90.f;

  // last path case 1: equal to the current nodes
  std::vector<geometry_msgs::Point> path_node_positions1;
  path_node_positions1.push_back(toPoint(node1));
  path_node_positions1.push_back(toPoint(tree_root));

  // last path case 2: different from current nodes
  Eigen::Vector3f node1_old(0.5f, 0.5f, 0.f);
  std::vector<geometry_msgs::Point> path_node_positions2;
  path_node_positions2.push_back(toPoint(node1_old));
  path_node_positions2.push_back(toPoint(tree_root));

  // WHEN: we calculate the cost of node 1 for two different old paths
  path_node_positions_ = path_node_positions1;
  double cost1 = treeCostFunction(1);
  path_node_positions_ = path_node_positions2;
  double cost2 = treeCostFunction(1);

  // THEN: The cost1 should be less than cost2, as in case 1 the node lies
  // closer to the path of the last iteration
  EXPECT_GT(cost2, cost1);
}

TEST_F(StarPlannerBasicTests, treeCostFunctionYawCost) {
  // GIVEN: a tree, a goal and the last path
  StarPlanner star_planner;
  Eigen::Vector3f goal(5.f, 0.f, 0.f);
  setGoal(toPoint(goal));
  tree_age_ = 1;

  // insert tree root
  Eigen::Vector3f tree_root(0.f, 0.f, 0.f);
  tree_.push_back(TreeNode(0, 0, tree_root));
  tree_.back().yaw_ = 90;  // drone looks straight ahead
  tree_.back().last_z_ = tree_.back().yaw_;

  // insert two nodes to both sides
  PolarPoint node1_pol(0, 110, 1);  // to the right
  PolarPoint node2_pol(0, 70, 1);   // to the left
  Eigen::Vector3f node1 = polarToCartesian(node1_pol, toPoint(tree_root));
  Eigen::Vector3f node2 = polarToCartesian(node2_pol, toPoint(tree_root));

  tree_.push_back(TreeNode(0, 1, node1));
  tree_.back().last_e_ = node1_pol.e;
  tree_.back().last_z_ = node1_pol.z;

  tree_.push_back(TreeNode(0, 1, node2));
  tree_.back().last_e_ = node2_pol.e;
  tree_.back().last_z_ = node2_pol.z;

  // last path straight ahead
  Eigen::Vector3f node_old(1.f, 0.f, 0.f);
  path_node_positions_.clear();
  path_node_positions_.push_back(toPoint(node_old));
  path_node_positions_.push_back(toPoint(tree_root));

  // WHEN: we calculate the cost for both nodes as the drone looks straight
  // ahead
  double cost1_straight = treeCostFunction(1);
  double cost2_straight = treeCostFunction(2);

  // WHEN: we calculate the cost for both nodes as the drone looks to the right
  tree_[0].yaw_ = 100;  // drone looks 10 degrees to the right
  tree_[0].last_z_ = tree_[0].yaw_;
  double cost1_right = treeCostFunction(1);
  double cost2_right = treeCostFunction(2);

  // WHEN: we calculate the cost for both nodes as the drone looks to the left
  tree_[0].yaw_ = 80;  // drone looks 10 degrees to the right
  tree_[0].last_z_ = tree_[0].yaw_;
  double cost1_left = treeCostFunction(1);
  double cost2_left = treeCostFunction(2);

  // THEN: case 1: drone looks straight ahead, nodes symmetrical to the left and
  // right should have same costs
  //       case 2: drone looks to the right, node to the right should be cheaper
  //       case 3: drone looks to the left, node to the left should be cheaper
  //       and costs should be symmetrical as well
  EXPECT_FLOAT_EQ(cost2_straight, cost1_straight);
  EXPECT_GT(cost2_right, cost1_right);
  EXPECT_GT(cost1_left, cost2_left);
  EXPECT_FLOAT_EQ(cost2_left, cost1_right);
  EXPECT_FLOAT_EQ(cost1_left, cost2_right);
}

TEST_F(StarPlannerBasicTests, treeCostFunctionSmoothingCost) {
  // GIVEN: a tree, a goal
  StarPlanner star_planner;
  path_node_positions_.clear();

  // insert tree root
  Eigen::Vector3f tree_root(0.f, 0.f, 0.f);
  tree_.push_back(TreeNode(0, 0, tree_root));
  tree_.back().last_z_ = 90;

  // insert first node (straight ahead)
  Eigen::Vector3f node1(1.f, 0.f, 0.f);
  tree_.push_back(TreeNode(0, 1, node1));
  tree_.back().last_e_ = 0.f;
  tree_.back().last_z_ = 90.f;

  // insert two more nodes with node 1 as origin
  PolarPoint node2_pol(0, 100, 1);
  PolarPoint node3_pol(0, 110, 1);
  Eigen::Vector3f node2 = polarToCartesian(node2_pol, toPoint(node1));
  Eigen::Vector3f node3 = polarToCartesian(node3_pol, toPoint(node1));

  tree_.push_back(TreeNode(1, 2, node2));
  tree_.back().last_e_ = node2_pol.e;
  tree_.back().last_z_ = node2_pol.z;

  tree_.push_back(TreeNode(1, 2, node3));
  tree_.back().last_e_ = node3_pol.e;
  tree_.back().last_z_ = node3_pol.z;

  // calculate two goal positions in direction of the nodes 2, 3
  PolarPoint goal2_pol(0, 100, 5);
  PolarPoint goal3_pol(0, 110, 5);
  Eigen::Vector3f goal2 = polarToCartesian(goal2_pol, toPoint(node1));
  Eigen::Vector3f goal3 = polarToCartesian(goal3_pol, toPoint(node1));

  // WHEN: we calculate the cost for nodes 2, 3
  setGoal(toPoint(goal2));
  tree_[0].yaw_ = 100;
  double cost2 = treeCostFunction(2);
  setGoal(toPoint(goal3));
  tree_[0].yaw_ = 110;
  double cost3 = treeCostFunction(3);

  // THEN: the path node with the more curved path (node 3) should be more
  // expensive
  EXPECT_GT(cost3, cost2);
}
