#ifndef GLOBAL_PLANNER_LOCAL_PLANNER_H
#define GLOBAL_PLANNER_LOCAL_PLANNER_H

#include <iostream>
#include <math.h> 

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>

#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_ros/conversions.h>

#define PI 3.14159265
#define alpha_res 10

float distance2d(geometry_msgs::Point a, geometry_msgs::Point b);


class Histogram { 

	int* array; 
	int m_width; 
	int h_width;

	public: 

		Histogram( int w, int h ): m_width(w), h_width(h), array( new int[ w * h ] ) {} 
		
		~Histogram() { 
			delete[] array; 
		} 
		
		int get( int x, int y ) const {
			std::cout << "x " << x << " y " << y << " m_width " << m_width << std::endl;
			std::cout << x + m_width * y << std::endl;
		 	return array[ index( x, y ) ]; 
		} 

		void set( int x, int y , int value) {
		 	array[ index( x, y ) ] = value; 
		}

		int setZero() {
		  	memset(array,0,m_width*h_width*sizeof(int)); 
		} 
		
	protected: 
		int index( int x, int y ) const { 
			return x + m_width * y; 
		} 
};

class LocalPlanner {

public:
	pcl::PointCloud<pcl::PointXYZ> final_cloud;
	octomap::Pointcloud octomapCloud;

	bool obstacle;
	Histogram polar_histogram; 

	geometry_msgs::Point pose, min, max, min_cache, max_cache, front, back, half_cache; // goal, obs, stop_pose;
	ros::Time last_pose_time;

	float min_x= 1.5, max_x= 1.5, min_y= 1.5, max_y= 1.5, min_z= 1.5, max_z=1.5; 
	float back_x= 0, front_x= 4.5, back_y= 0.6, front_y= 0.6, back_z= 0.6, front_z=0.6;
	float min_cache_x= 2.5, max_cache_x= 2.5, min_cache_y= 2.5, max_cache_y= 2.5, min_cache_z= 2.5, max_cache_z=2.5; 
	double previous_pose_x,previous_pose_z;
	int grid_length = 360/alpha_res;

	LocalPlanner();
	~LocalPlanner();
	
	void setPose(const geometry_msgs::PoseStamped input);
	void setLimits();
	void filterPointCloud(pcl::PointCloud<pcl::PointXYZ>& );
	void createPolarHistogram();

};


#endif // GLOBAL_PLANNER_LOCAL_PLANNER_H