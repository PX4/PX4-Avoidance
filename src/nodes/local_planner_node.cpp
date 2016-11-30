#include "local_planner_node.h"

LocalPlannerNode::LocalPlannerNode() {

	pointcloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/omi_cam/point_cloud", 1, &LocalPlannerNode::pointCloudCallback, this);
	pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, &LocalPlannerNode::positionCallback, this);
	// /mavros/local_position/pose  ground_truth/pose
}

LocalPlannerNode::~LocalPlannerNode(){}

void LocalPlannerNode::positionCallback(const geometry_msgs::PoseStamped input){
	std::cout << "set pose call" << std::endl;
	local_planner.setPose(input);
}

void LocalPlannerNode::pointCloudCallback(const sensor_msgs::PointCloud2 input){

	pcl::PointCloud<pcl::PointXYZ> complete_cloud;
    sensor_msgs::PointCloud2 pc2cloud_world;


    tf_listener_.waitForTransform("/world", input.header.frame_id, input.header.stamp, ros::Duration(1.0));
    tf::StampedTransform transform;
    tf_listener_.lookupTransform("/world", input.header.frame_id, input.header.stamp, transform);
    pcl_ros::transformPointCloud("/world", transform, input, pc2cloud_world);
    pcl::fromROSMsg(pc2cloud_world, complete_cloud); 
    local_planner.filterPointCloud(complete_cloud);

   // if(local_planner.obstacleAhead() && local_planner.init!=0 && !brake_test) {
	local_planner.createPolarHistogram();
//	}


}

int main(int argc, char** argv) {
  ros::init(argc, argv, "local_planner_node");
  LocalPlannerNode local_planner_node;
  ros::Duration(2).sleep(); 
  ros::spin();
  
  return 0;
}


 	
