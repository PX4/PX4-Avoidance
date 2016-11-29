#include "local_planner.h"

LocalPlanner::LocalPlanner() {}

LocalPlanner::~LocalPlanner() {}

void LocalPlanner::filterPointCloud(pcl::PointCloud<pcl::PointXYZ>& complete_cloud) {
	pcl::PointCloud<pcl::PointXYZ>::iterator pcl_it;
  	pcl::PointCloud<pcl::PointXYZ> front_cloud;
  	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  	final_cloud.points.clear();

	for (pcl_it = complete_cloud.begin(); pcl_it != complete_cloud.end(); ++pcl_it) {
      // Check if the point is invalid
    	if (!std::isnan (pcl_it->x) && !std::isnan (pcl_it->y) && !std::isnan (pcl_it->z)) {
    		std::cout << "valid point" << std::endl;
   			if((pcl_it->x)<max_cache.x&&(pcl_it->x)>min_cache.x&&(pcl_it->y)<max_cache.y&&(pcl_it->y)>min_cache.y&&(pcl_it->z)<max_cache.z&&(pcl_it->z)>min_cache.z) {
        		octomapCloud.push_back(pcl_it->x, pcl_it->y, pcl_it->z);
        		std::cout << "octomapCloud push back" << std::endl;
      		}
      
      		if((pcl_it->x)<front.x&&(pcl_it->x)>back.x&&(pcl_it->y)<front.y&&(pcl_it->y)>back.y&&(pcl_it->z)<front.z&&(pcl_it->z)>back.z) {
        		front_cloud.points.push_back(pcl::PointXYZ(pcl_it->x,pcl_it->y,pcl_it->z));
        		std::cout << "front_cloud push back" << std::endl;
      		}
    	}
	}

	if(front_cloud.points.size()>1) {    
    	obstacle = true;

    	octomap::Pointcloud::iterator oc_it;
		for (oc_it = octomapCloud.begin(); oc_it != octomapCloud.end(); ++oc_it) {
			std::cout << "iterator" << std::endl;
      		if((oc_it->x())<max.x&&(oc_it->x())>min.x&&(oc_it->y())<max.y&&(oc_it->y())>min.y&&(oc_it->z())<max.z&&(oc_it->z())>min.z) {
        		cloud->points.push_back(pcl::PointXYZ(oc_it->x(),oc_it->y(),oc_it->z()));
        		std::cout << "cloud->points push back" << std::endl;
      		}
   		}
 
  		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  		sor.setInputCloud(cloud);
  		sor.setMeanK (5);
  		sor.setStddevMulThresh (0.5);
  		sor.filter(final_cloud);
  		std::cout << "outliers removed" << std::endl;

  	}
  	else
    	obstacle = false;
   
    ROS_INFO(" Cloud transformed");

    final_cloud.header.stamp =  complete_cloud.header.stamp;
    final_cloud.header.frame_id = complete_cloud.header.frame_id;
    final_cloud.width = final_cloud.points.size();
    final_cloud.height = 1; 
}