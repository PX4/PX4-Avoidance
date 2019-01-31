#include <gtest/gtest.h>

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

  void SetUp() override {
    ros::Time::init();

    avoidance::LocalPlannerNodeConfig config =
        avoidance::LocalPlannerNodeConfig::__getDefault__();
    config.childs_per_node_ = 2;
    config.n_expanded_nodes_ = 10;
    star_planner.dynamicReconfigureSetStarParams(config, 1);

    geometry_msgs::PoseStamped position;
    position.pose.position.x = 1.2f;
    position.pose.position.y = 0.4f;
    position.pose.position.z = 4.0f;
    position.pose.orientation.x = 0.0f;
    position.pose.orientation.y = 0.0f;
    position.pose.orientation.z = 0.0f;
    position.pose.orientation.w = 0.0f;

    geometry_msgs::Point goal;
    goal.x = 2.0f;
    goal.y = 14.0f;
    goal.z = 4.0f;

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
    star_planner.setFOV(270.0, 45.0);
    star_planner.setReprojectedPoints(reprojected_points,
                                      reprojected_points_age);
    star_planner.setPose(position, 0.0);
    star_planner.setGoal(goal);
    star_planner.setCloud(cloud);
  }
  void TearDown() override {}
};

TEST_F(StarPlannerTests, buildTree) {
  // GIVEN: a vehicle position, a goal, and an obstacle in between the stright
  // line position-goal

  // WHEN: we build the tree
  star_planner.buildLookAheadTree();

  // THEN: no cell position should be in the obstacle coordinates
  std::vector<TreeNode> tree_truth;
  TreeNode n0(0, 0, Eigen::Vector3f(1.2, 0.4, 4));
  TreeNode n1(0, 1, Eigen::Vector3f(2.18633, 0.55622, 3.94766));
  TreeNode n2(1, 2, Eigen::Vector3f(3.17267, 0.71244, 3.89532));
  TreeNode n3(2, 3, Eigen::Vector3f(4.159, 0.86866, 3.842992));

  TreeNode n4(3, 4, Eigen::Vector3f(5.14534, 1.02488, 3.79065));
  TreeNode n5(4, 5, Eigen::Vector3f(6.13167, 1.1811, 3.73832));
  TreeNode n6(5, 6, Eigen::Vector3f(7.11801, 1.33732, 3.68598));
  TreeNode n7(6, 7, Eigen::Vector3f(8.10434, 1.49354, 3.63364));
  TreeNode n8(7, 8, Eigen::Vector3f(9.09068, 1.64976, 3.58131));
  TreeNode n9(8, 9, Eigen::Vector3f(10.07701, 1.80598, 3.52897));
  TreeNode n10(9, 10, Eigen::Vector3f(11.063347, 1.9622, 3.47664));

  tree_truth = {n0, n1, n2, n3, n4, n5, n6, n7, n8, n9, n10};

  for (size_t i = 0, j = 0;
       i < star_planner.tree_.size(), j < tree_truth.size(); i++, j++) {
    Eigen::Vector3f n = star_planner.tree_[i].getPosition();
    Eigen::Vector3f t = tree_truth[j].getPosition();
    ASSERT_NEAR(t.x(), n.x(), .00001) << "Node number: " << i;
    ASSERT_NEAR(t.y(), n.y(), .00001) << "Node number: " << i;
    ASSERT_NEAR(t.z(), n.z(), .00001) << "Node number: " << i;
    bool node_inside_obstacle =
        n.x() > obstacle_min_x && n.x() < obstacle_max_x &&
        n.y() > obstacle_y - 0.1f && n.y() < obstacle_y + 0.1 &&
        n.z() > 4.0f - obstacle_half_height &&
        n.z() < 4.0f + obstacle_half_height;
    EXPECT_FALSE(node_inside_obstacle);
  }
}
