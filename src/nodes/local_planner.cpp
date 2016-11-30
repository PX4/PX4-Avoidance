#include "local_planner.h"

LocalPlanner::LocalPlanner(): polar_histogram(grid_length, grid_length) {}

LocalPlanner::~LocalPlanner() {}

void LocalPlanner::setPose(const geometry_msgs::PoseStamped input) {
  pose.x = input.pose.position.x ;
  pose.y = input.pose.position.y ;
  pose.z = input.pose.position.z ; 

  std::cout << "pose:" << pose.x << pose.y << pose.z << std::endl;
//  setVelocity(input.header.stamp);
  previous_pose_x = pose.x ;
  previous_pose_z = pose.z ;
  last_pose_time = input.header.stamp;
 // setGoal();
  setLimits();

 // octomapCloud.crop(octomap::point3d(min_cache.x,min_cache.y,min_cache.z),octomap::point3d(half_cache.x,half_cache.y,half_cache.z)); 
}

void LocalPlanner::setLimits() { 
  //front_x = wavefront_param*velocity_x;
  //if(front_x<=4.5)
  //  front_x = 4.5;
  min.x = pose.x- min_x;
  min.y = pose.y- min_y;
  min.z = pose.z- min_z;
  max.x = pose.x + max_x;
  max.y = pose.y + max_y;
  max.z= pose.z + max_z;
  front.x = pose.x + front_x;
  front.y = pose.y + front_y;
  front.z = pose.z + front_z;
  back.x = pose.x - back_x;
  back.y = pose.y - back_y;
  back.z = pose.z - back_z;
  min_cache.x = pose.x - min_cache_x;
  min_cache.y = pose.y - min_cache_y;
  min_cache.z = pose.z - min_cache_z;
  max_cache.x = pose.x + max_cache_x;
  max_cache.y = pose.y + max_cache_y;
  max_cache.z = pose.z + max_cache_z;
  half_cache.x = pose.x;
  half_cache.y = pose.y + max_cache_y;
  half_cache.z = pose.z + max_cache_z;
  
}

void LocalPlanner::filterPointCloud(pcl::PointCloud<pcl::PointXYZ>& complete_cloud) {
	pcl::PointCloud<pcl::PointXYZ>::iterator pcl_it;
  	pcl::PointCloud<pcl::PointXYZ> front_cloud;
  	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  	final_cloud.points.clear();

  	//std::cout << "complete cloud size " << complete_cloud.points.size();

	for (pcl_it = complete_cloud.begin(); pcl_it != complete_cloud.end(); ++pcl_it) {
      // Check if the point is invalid
    	if (!std::isnan (pcl_it->x) && !std::isnan (pcl_it->y) && !std::isnan (pcl_it->z)) {
   			if((pcl_it->x)<max_cache.x&&(pcl_it->x)>min_cache.x&&(pcl_it->y)<max_cache.y&&(pcl_it->y)>min_cache.y&&(pcl_it->z)<max_cache.z&&(pcl_it->z)>min_cache.z) {
        		octomapCloud.push_back(pcl_it->x, pcl_it->y, pcl_it->z);
        		//std::cout << "octomapCloud push back" << std::endl;
      		}
      
      		if((pcl_it->x)<front.x&&(pcl_it->x)>back.x&&(pcl_it->y)<front.y&&(pcl_it->y)>back.y&&(pcl_it->z)<front.z&&(pcl_it->z)>back.z) {
        		front_cloud.points.push_back(pcl::PointXYZ(pcl_it->x,pcl_it->y,pcl_it->z));
        		//std::cout << "front_cloud push back" << std::endl;
      		}
    	}
	}

	if(front_cloud.points.size()>1) {    
    	obstacle = true;

    	octomap::Pointcloud::iterator oc_it;
		for (oc_it = octomapCloud.begin(); oc_it != octomapCloud.end(); ++oc_it) {
			//std::cout << "iterator" << std::endl;
      		if((oc_it->x())<max.x&&(oc_it->x())>min.x&&(oc_it->y())<max.y&&(oc_it->y())>min.y&&(oc_it->z())<max.z&&(oc_it->z())>min.z) {
        		cloud->points.push_back(pcl::PointXYZ(oc_it->x(),oc_it->y(),oc_it->z()));
        		//std::cout << "cloud->points push back" << std::endl;
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
    std::cout << "final cloud width " << final_cloud.width << std::endl;
    final_cloud.height = 1; 
}

float distance2d(geometry_msgs::Point a, geometry_msgs::Point b) {
   return sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) +(a.z-b.z)*(a.z-b.z) );
}

void LocalPlanner::createPolarHistogram() {  

	float bbx_rad = (max.x-min.x)*sqrt(2)/2; 
  	float dist;
 
  	polar_histogram.setZero();

  	std::cout << "polar_histogram created " << std::endl;
  	std::cout << polar_histogram.get(20,23) << std::endl;
  	std::cout << polar_histogram.get(35,35) << std::endl;
 

  	pcl::PointCloud<pcl::PointXYZ>::const_iterator it;
    
  	for( it = final_cloud.begin(); it != final_cloud.end(); ++it) {   
    	geometry_msgs::Point temp; 
    	temp.x= it->x;
    	temp.y = it->y;
    	temp.z = it->z;
    	dist = distance2d(pose,temp);

    	std::cout << dist << std::endl;
   
    	if(dist < bbx_rad) { 
      		int beta_z = floor((atan2(temp.x-pose.x,temp.y-pose.y)*180.0/PI)); //azimuthal angle
      		int beta_e = floor((atan2(temp.z-pose.z,sqrt((temp.x-pose.x)*(temp.x-pose.x)+(temp.y-pose.y)*(temp.y-pose.y)))*180.0/PI));//elevation angle
    
      		beta_z = beta_z + (alpha_res - beta_z%alpha_res);
      		beta_e = beta_e + (alpha_res - beta_e%alpha_res); 
      
      		int e = (180+beta_e)/alpha_res - 1 ;
      		int z = (180+beta_z)/alpha_res - 1 ;

      		std::cout << "e " << e << " z " << z  << std::endl;
      		std::cout << polar_histogram.get(e,z) << std::endl;

      		polar_histogram.set(e,z,polar_histogram.get(e,z)+1);
    	}
 	}
}
