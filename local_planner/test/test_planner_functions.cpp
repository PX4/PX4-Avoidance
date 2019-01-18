#include <gtest/gtest.h>

#include "../src/nodes/planner_functions.h"

using namespace avoidance;



TEST(PlannerFunctions, initGridCells){
	//GIVEN: an empty grid cell
	nav_msgs::GridCells init_cell = {};
	init_cell.cell_width = 1;
	init_cell.cell_height = 1;

	//WHEN: initilaize the cell's hieght and width
	initGridCells(&init_cell);
	//THEN: the cell height and width should be ..
	EXPECT_EQ(6, init_cell.cell_width);
	EXPECT_EQ(6, init_cell.cell_height);
	}



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
    	  EXPECT_FLOAT_EQ(0.0, histogram_output.get_bin(e, z));
      }
    }
}


TEST(PlannerFunctions, DISABLED_generateNewHistogramSpecificCells) {
  // GIVEN: a pointcloud with an object of one cell size
	Histogram histogram_output = Histogram(ALPHA_RES);
	geometry_msgs::PoseStamped location;
	location.pose.position.x = 0;
	location.pose.position.y = 0;
	location.pose.position.z = 0;
	double distance  = 1.0;

	std::vector<float> e_angle_filled = {-90, -30, 0, 20, 40, 90};
	std::vector<float> z_angle_filled = {-180, -50, 0, 59, 100, 175};
	std::vector<geometry_msgs::Point> middle_of_cell;

	for(auto i : e_angle_filled){
		for(auto j : z_angle_filled){
		   middle_of_cell.push_back(fromPolarToCartesian(i, j, distance, location.pose.position));
		}
	}

	pcl::PointCloud<pcl::PointXYZ> cloud;
	for(int i = 0; i < middle_of_cell.size(); i++){
		  cloud.push_back(pcl::PointXYZ(middle_of_cell[i].x, middle_of_cell[i].y, middle_of_cell[i].z));
	}


  // WHEN: we build a histogram
    generateNewHistogram(histogram_output, cloud, location);


  // THEN: the filled cells in the histogram should be one and the others be zeros

	std::vector<int> e_index;
	std::vector<int> z_index;
	for(auto i : e_angle_filled){
		e_index.push_back(elevationAngletoIndex(i, ALPHA_RES));
    }

	for(auto i : z_angle_filled){
		z_index.push_back(azimuthAngletoIndex(i, ALPHA_RES));
    }

	for (int e = 0; e < GRID_LENGTH_E; e++) {
	  for (int z = 0; z < GRID_LENGTH_Z; z++) {
		  bool e_found = std::find(e_index.begin(), e_index.end(), e) != e_index.end();
		  bool z_found = std::find(z_index.begin(), z_index.end(), z) != z_index.end();
	      if(e_found && z_found){
	   	    EXPECT_FLOAT_EQ(1.0, histogram_output.get_bin(e, z)) << z <<", "<<e;
	      }else{
	    	EXPECT_FLOAT_EQ(0.0, histogram_output.get_bin(e, z)) << z <<", "<<e;
	      }

	  }
	}
}
