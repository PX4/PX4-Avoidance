#include <gtest/gtest.h>

#include "../src/nodes/planner_functions.h"
#include "../src/nodes/common.h"

using namespace avoidance;

TEST(PlannerFunctions, generateNewHistogramEmpty) {
  // GIVEN: an empty pointcloud
  pcl::PointCloud<pcl::PointXYZ> empty_cloud;
  Histogram histogram_output = Histogram(ALPHA_RES);
  geometry_msgs::PoseStamped location;
  location.pose.position.x = 0;
  location.pose.position.y = 0;
  location.pose.position.z = 0;

  // WHEN: we build a histogram
  generateNewHistogram(histogram_output, empty_cloud, location);

  // THEN: the histogram should be all zeros
  for (int e = 0; e < GRID_LENGTH_E; e++) {
    for (int z = 0; z < GRID_LENGTH_Z; z++) {
      EXPECT_DOUBLE_EQ(0.0, histogram_output.get_bin(e, z));
    }
  }
}

TEST(PlannerFunctions, generateNewHistogramSpecificCells) {
  // GIVEN: a pointcloud with an object of one cell size
  Histogram histogram_output = Histogram(ALPHA_RES);
  geometry_msgs::PoseStamped location;
  location.pose.position.x = 0;
  location.pose.position.y = 0;
  location.pose.position.z = 0;
  double distance = 1.0;

  std::vector<double> e_angle_filled = {-90, -30, 0, 20, 40, 90};
  std::vector<double> z_angle_filled = {-180, -50, 0, 59, 100, 175};
  std::vector<Eigen::Vector3f> middle_of_cell;

  for (int i = 0; i < e_angle_filled.size(); i++) {
    for (int j = 0; j < z_angle_filled.size(); j++) {
      middle_of_cell.push_back(fromPolarToCartesian(e_angle_filled[i],
                                                    z_angle_filled[j], distance,
                                                    location.pose.position));
    }
  }

  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (int i = 0; i < middle_of_cell.size(); i++) {
    for (int j = 0; j < 1; j++) {
      cloud.push_back(toXYZ(middle_of_cell[i]));
    }
  }

  // WHEN: we build a histogram
  generateNewHistogram(histogram_output, cloud, location);

  // THEN: the filled cells in the histogram should be one and the others be
  // zeros

  std::vector<int> e_index;
  std::vector<int> z_index;
  for (int i = 0; i < e_angle_filled.size(); i++) {
    e_index.push_back(elevationAngletoIndex((int)e_angle_filled[i], ALPHA_RES));
    z_index.push_back(azimuthAngletoIndex((int)z_angle_filled[i], ALPHA_RES));
  }

  for (int e = 0; e < GRID_LENGTH_E; e++) {
    for (int z = 0; z < GRID_LENGTH_Z; z++) {
      bool e_found =
          std::find(e_index.begin(), e_index.end(), e) != e_index.end();
      bool z_found =
          std::find(z_index.begin(), z_index.end(), z) != z_index.end();
      if (e_found && z_found) {
        EXPECT_DOUBLE_EQ(1.0, histogram_output.get_bin(e, z)) << z << ", " << e;
      } else {
        EXPECT_DOUBLE_EQ(0.0, histogram_output.get_bin(e, z)) << z << ", " << e;
      }
    }
  }
}
