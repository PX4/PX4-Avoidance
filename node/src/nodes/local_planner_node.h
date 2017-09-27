#ifndef GLOBAL_PLANNER_LOCAL_PLANNER_NODE_H
#define GLOBAL_PLANNER_LOCAL_PLANNER_NODE_H

#include <iostream>
#include <math.h> 

#include <boost/bind.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h> // transformPointCloud
#include <pcl_conversions/pcl_conversions.h>  // fromROSMsg
#include <pcl_ros/point_cloud.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <tf/transform_listener.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include <nav_msgs/GridCells.h>
#include <nav_msgs/Path.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h> 

#include <Eigen/Core>

#include "local_planner.h"

#include "avoidance/common_ros.h"



class LocalPlannerNode{

public: 
	LocalPlannerNode();
	~LocalPlannerNode();

private:
	ros::NodeHandle nh_;
	LocalPlanner local_planner;

	dynamic_reconfigure::Server<avoidance::LocalPlannerNodeConfig> server_;

	nav_msgs::Path path_actual;

	int i = 0 ;

	//subscribers
	ros::Subscriber pointcloud_sub_ ;
	ros::Subscriber pose_sub_ ; 
	ros::Subscriber velocity_sub_ ;
	ros::Subscriber clicked_point_sub_;
	ros::Subscriber clicked_goal_sub_;


	//publishers
	ros::Publisher local_pointcloud_pub_;
	ros::Publisher front_pointcloud_pub_;
	ros::Publisher cached_pointcloud_pub_ ;
	ros::Publisher marker_pub_;
	ros::Publisher waypoint_pub_;
    ros::Publisher path_pub_;
    ros::Publisher mavros_waypoint_pub_;
  	ros::Publisher current_waypoint_pub_;
  	ros::Publisher marker_rejected_pub_;
  	ros::Publisher marker_blocked_pub_;
  	ros::Publisher marker_candidates_pub_;
  	ros::Publisher marker_selected_pub_;
  	ros::Publisher marker_extended_pub_;
  	ros::Publisher marker_goal_pub_;
  	ros::Publisher marker_normal_pub_;

    tf::TransformListener tf_listener_;
    
    std::vector<float> algo_time;

    void dynamicReconfigureCallback(avoidance::LocalPlannerNodeConfig & config, uint32_t level);
    void positionCallback(const geometry_msgs::PoseStamped msg);
	void pointCloudCallback(const sensor_msgs::PointCloud2 msg);
	void velocityCallback(const geometry_msgs::TwistStamped msg);
	void readParams();
	void publishAll();
	void publishPath(const geometry_msgs::PoseStamped msg);
	void initMarker(visualization_msgs::MarkerArray *marker, nav_msgs::GridCells path, float red, float green , float blue);
	void publishMarkerBlocked();
	void publishMarkerRejected();
	void publishMarkerCandidates();
	void publishMarkerSelected();
	void publishMarkerExtended();
	void clickedPointCallback(const geometry_msgs::PointStamped & msg);
	void clickedGoalCallback(const geometry_msgs::PoseStamped & msg);
	void printPointInfo(double x, double y, double z);
	void publishGoal();
	void publishNormalToPowerline();
	

};


#endif // GLOBAL_PLANNER_LOCAL_PLANNER_NODE_H
