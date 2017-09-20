#include <stdio.h>
#include <unistd.h>
#include <sstream>

#include <ros/ros.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

int main(int argc, char **argv)
{


	ros::init(argc, argv, "uvc_ros_driver");
	ros::NodeHandle nh("~");  // private nodehandle

	ros::Time last_time = ros::Time::now();

	// UserData user_data;

	// nh.param<bool>("hflip", user_data.hflip, false);
	// nh.param<bool>("serialconfig", user_data.serialconfig, false);
	//nh.getParam("serialconfig",user_data.serialconfig);

	//ros::Publisher serial_nr_pub = nh.advertise<std_msgs::String>("/vio_sensor/device_serial_nr", 1, true);

	ROS_INFO("Mapping node launched");

	// Subscribe to image stream

	// Build integrated point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	// Reproject into 3D in world coordinates
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(50);
	sor.setStddevMulThresh(1.0);
	sor.filter(*cloud_filtered);

	// Estimate safe avoidance path

	// Send position command out


	ros::spin();

	ROS_INFO("Mapping node exited");

	return 0;
}