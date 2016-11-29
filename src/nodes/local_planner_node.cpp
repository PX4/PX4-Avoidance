#include "local_planner_node.h"

LocalPlannerNode::LocalPlannerNode() {

	pointcloud_sub_ = nh_.subscribe("/omi_cam/point_cloud", 1, &LocalPlannerNode::pointCloudCallback, this);

}

LocalPlannerNode::~LocalPlannerNode(){}

void LocalPlannerNode::pointCloudCallback(const sensor_msgs::PointCloud2 input){

	 pcl::PointCloud<pcl::PointXYZ> complete_cloud;
     sensor_msgs::PointCloud2 pc2cloud_world;
      
     std::cout << "point cloud callback" << std::endl; 
     std::cout << "frame id " << input.header.frame_id << " header stamp " << input.header.stamp << std::endl;

     tf_listener->waitForTransform("/world", input.header.frame_id, input.header.stamp, ros::Duration(1.0));
    /* std::cout << "wait for transform " << std::endl;
     pcl_ros::transformPointCloud("/world", input, pc2cloud_world, *tf_listener);
     
     std::cout << "pointcloud transformed" << std::endl; 

     pcl::fromROSMsg(pc2cloud_world, complete_cloud); */

	  //local_planner.filterPointCloud(complete_cloud);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "local_planner_node");
  LocalPlannerNode local_planner_node;
  ros::Duration(2).sleep(); 
  ros::spin();
  
  return 0;
}


 