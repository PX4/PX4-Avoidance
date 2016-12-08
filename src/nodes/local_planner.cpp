#include "local_planner.h"

LocalPlanner::LocalPlanner() {
    current_goal.vector.x = goal_x_param;
    current_goal.vector.y = goal_y_param;
    current_goal.vector.z = goal_z_param;
    current_goal.header.frame_id = "/world";
    current_goal.header.stamp = ros::Time::now();
   // double strat_yaw = 0;
   // current_goal.pose.orientation = tf::createQuaternionMsgFromYaw(strat_yaw);
    last_goal = current_goal;

}

LocalPlanner::~LocalPlanner() {}

void LocalPlanner::setPose(const geometry_msgs::PoseStamped msg) {
  pose.header = msg.header;
  pose.pose.position = msg.pose.position;
  pose.pose.orientation = msg.pose.orientation;
  //pose.x = input.pose.position.x ;
  //pose.y = input.pose.position.y ;

  //pose.z = input.pose.position.z ; 
  curr_yaw = tf::getYaw(msg.pose.orientation);

  setVelocity(msg.header.stamp);
  previous_pose_x = pose.pose.position.x;
  previous_pose_y = pose.pose.position.y;
  previous_pose_z = pose.pose.position.z;
  last_pose_time = msg.header.stamp;
  
  setLimits();

 // octomapCloud.crop(octomap::point3d(min_cache.x,min_cache.y,min_cache.z),octomap::point3d(half_cache.x,half_cache.y,half_cache.z)); 
}

void LocalPlanner::setVelocity(ros::Time current) {

//  double dt = (current-last_pose_time).toSec();
  velocity_x = curr_vel.twist.linear.x;
  velocity_y = curr_vel.twist.linear.y;
  velocity_z = curr_vel.twist.linear.z;
  fall_height = velocity_x*velocity_x/(2*9.81);
}

void LocalPlanner::setLimits() { 
//  front_x = wavefront_param*velocity_x;
 // if(front_x<=4.5)
 //   front_x = 4.5;
  min.x = pose.pose.position.x- min_x;
  min.y = pose.pose.position.y- min_y;
  min.z = pose.pose.position.z- min_z;
  max.x = pose.pose.position.x + max_x;
  max.y = pose.pose.position.y + max_y;
  max.z= pose.pose.position.z + max_z;
  front.x = pose.pose.position.x + front_x;
  front.y = pose.pose.position.y + front_y;
  front.z = pose.pose.position.z + front_z;
  back.x = pose.pose.position.x - back_x;
  back.y = pose.pose.position.y - back_y;
  back.z = pose.pose.position.z - back_z;
  min_cache.x = pose.pose.position.x - min_cache_x;
  min_cache.y = pose.pose.position.y - min_cache_y;
  min_cache.z = pose.pose.position.z - min_cache_z;
  max_cache.x = pose.pose.position.x + max_cache_x;
  max_cache.y = pose.pose.position.y + max_cache_y;
  max_cache.z = pose.pose.position.z + max_cache_z;
  half_cache.x = pose.pose.position.x;
  half_cache.y = pose.pose.position.y + max_cache_y;
  half_cache.z = pose.pose.position.z + max_cache_z;


 // printf("back %f front %f \n",back.x, front.x);
  
}

void LocalPlanner::setGoal() {
  	goal.x = goal_x_param;
  	goal.y = goal_y_param;
  	goal.z = goal_z_param;


    ROS_INFO("===== Set Goal ======: [%f, %f, %f].",
           goal.x,
           goal.y,
           goal.z);
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




  	}
  	else {
    	obstacle = false;
    }
   
    ROS_INFO(" Cloud transformed");

    final_cloud.header.stamp =  complete_cloud.header.stamp;
    final_cloud.header.frame_id = complete_cloud.header.frame_id;
    final_cloud.width = final_cloud.points.size();
    std::cout << "final cloud width " << final_cloud.width << " obstacle " << obstacle << std::endl;
    final_cloud.height = 1; 

    pcl::toROSMsg(final_cloud, final_cloud_pc2);

}

float distance2d(geometry_msgs::Point a, geometry_msgs::Point b) {
   return sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) +(a.z-b.z)*(a.z-b.z) );
}

bool LocalPlanner::obstacleAhead() { 
  	float dist_y =  abs(pose.pose.position.y - goal.y);
  	float dist_z =  abs(pose.pose.position.z - goal.z);
    
    if(obstacle) { //} && dist_y>path_y_tolerance_param &&  dist_z >path_z_tolerance_param){
      first_brake = true; 
          obs.x = pose.pose.position.x+front_x ;
          obs.y = pose.pose.position.y;
          obs.z = pose.pose.position.z;
          stop_pose.x = pose.pose.position.x;
          stop_pose.y = pose.pose.position.y;
          stop_pose.z = pose.pose.position.z;
    }
    else{
      first_brake = false;
      return false;
    }

/*	if(obstacle || dist_y>path_y_tolerance_param ||  dist_z >path_z_tolerance_param || first_brake) { 
    	if(!first_brake) {   
      		first_brake = true; 
      		obs.x = pose.x+front_x ;
      		obs.y = pose.y;
      		obs.z = pose.z;
      		stop_pose.x = pose.x;
      		stop_pose.y = pose.y;
      		stop_pose.z = pose.z;
    	}
		return true;
  	}
	else {
		first_brake = false;
    	return false;
   	}  */
}

void LocalPlanner::createPolarHistogram() {  

	float bbx_rad = (max.x-min.x)*sqrt(2)/2; 
  	float dist;
 
  	polar_histogram.setZero();

  	pcl::PointCloud<pcl::PointXYZ>::const_iterator it;
    
  	for( it = final_cloud.begin(); it != final_cloud.end(); ++it) {   
    	geometry_msgs::Point temp; 
    	temp.x= it->x;
    	temp.y = it->y;
    	temp.z = it->z;
    	dist = distance2d(pose.pose.position,temp);
   
    	if(dist < bbx_rad) { 
      		int beta_z = floor((atan2(temp.x-pose.pose.position.x,temp.y-pose.pose.position.y)*180.0/PI)); //azimuthal angle
      		int beta_e = floor((atan2(temp.z-pose.pose.position.z,sqrt((temp.x-pose.pose.position.x)*(temp.x-pose.pose.position.x)+(temp.y-pose.pose.position.y)*(temp.y-pose.pose.position.y)))*180.0/PI));//elevation angle
    
      		beta_z = beta_z + (alpha_res - beta_z%alpha_res);
      		beta_e = beta_e + (alpha_res - beta_e%alpha_res); 
      
      		int e = (180+beta_e)/alpha_res - 1;
      		int z = (180+beta_z)/alpha_res - 1;

      		polar_histogram.set(e,z,polar_histogram.get(e,z)+1);
    	}
 	}
}

void LocalPlanner::findFreeDirections() {

  int n = floor(40/alpha_res); //n+1 - angular window size
  int a = 0, b = 0;
  int sum = 0, pathID = 1;
  bool free = true;
  bool corner = false;
  geometry_msgs::Point p;

  path_candidates.cells.clear();
  path_candidates.header.stamp = ros::Time::now();
  path_candidates.header.frame_id = "/world";
  path_candidates.cell_width = alpha_res;
  path_candidates.cell_height = alpha_res;

  path_rejected.cells.clear();
  path_rejected.header.stamp = ros::Time::now();
  path_rejected.header.frame_id = "/world";
  path_rejected.cell_width = alpha_res;
  path_rejected.cell_height = alpha_res;

  path_blocked.cells.clear();
  path_blocked.header.stamp = ros::Time::now();
  path_blocked.header.frame_id = "/world";
  path_blocked.cell_width = alpha_res;
  path_blocked.cell_height = alpha_res;

  path_selected.cells.clear();
  path_selected.header.stamp = ros::Time::now();
  path_selected.header.frame_id = "/world";
  path_selected.cell_width = alpha_res;
  path_selected.cell_height = alpha_res;


  for(int e= 0; e<grid_length; e++) {
    for(int z= 0; z<grid_length; z++) {  
        for(int i=e-n; i<=e+n; i++) {
            for(int j=z-n;j<=z+n;j++) {
                free = true;
                corner = false;

                //Case 1 - i < 0
                if(i<0 && j>=0 && j<grid_length) {
                    a = -i;
                    b = grid_length-j-1;
                }
                //Case 2 - j < 0
                else if(j<0 && i>=0 && i<grid_length) {
                    b = j+grid_length;
                } 
                //Case 3 - i >= grid_length
                else if(i>=grid_length && j>=0 && j<grid_length) {
                    a = grid_length-(i%(grid_length-1));
                    b = grid_length-j-1;
                }
                //Case 4 - j >= grid_length
                else if(j>=grid_length && i>=0 && i<grid_length) {
                    b = j%grid_length;
                }
                else if( i>=0 && i<grid_length && j>=0 && j<grid_length) {
                    a = i;
                    b = j;
                }
                else {
                    corner = true;
                }

                if(!corner) {
                    if(polar_histogram.get(a,b) != 0) {
                    	free = false;
                    // ROS_INFO(" in the path loop %f %d %d", Hist_polar_binary[a][b] , a, b ) ;
                     	break;
                    } 
                }
            }
            
            if(!free)
                break;
            }
               
           	if(free) {		
            	p.x = e*alpha_res+alpha_res-180;  
            	p.y = z*alpha_res+alpha_res-180;
            	p.z = 0;
            	path_candidates.cells.push_back(p);  
            }
            else if(!free && polar_histogram.get(e,z) != 0) {
            	p.x = e*alpha_res+alpha_res-180;  
            	p.y = z*alpha_res+alpha_res-180;
            	p.z = 0;
            	path_rejected.cells.push_back(p);  
            	std::cout << "x " << p.x << " y " << p.y << " rejected" << std::endl;    
           	}
           	else {
            	p.x = e*alpha_res+alpha_res-180;  
            	p.y = z*alpha_res+alpha_res-180;
            	p.z = 0;
            	path_blocked.cells.push_back(p);   
            	std::cout << "x " << p.x << " y " << p.y << " blocked" << std::endl;   
            }
      	} 
    }

    ROS_INFO(" Path_candidates calculated");


   
}

geometry_msgs::Vector3Stamped LocalPlanner::getWaypointFromAngle(int e, int z) { 
  	geometry_msgs::Vector3Stamped waypoint;
  	waypoint.header.stamp = ros::Time::now();
  	waypoint.header.frame_id = "/world";
  	if(velocity_x > 1.4) {
    	rad = waypoint_radius_param*velocity_x;
      	if(rad==0)
        	rad=1;
    	}
  	else
    	rad = 1;

  	waypoint.vector.x = pose.pose.position.x+ rad*cos(e*(PI/180))*sin(z*(PI/180));
  	waypoint.vector.y = pose.pose.position.y+ rad*cos(e*(PI/180))*cos(z*(PI/180));
  	waypoint.vector.z = pose.pose.position.z+ rad*sin(e*(PI/180));

  	return waypoint;
}

double LocalPlanner::costFunction(int e, int z) {

  double cost;
  int goal_z = floor(atan2(goal.x-pose.pose.position.x,goal.y-pose.pose.position.y)*180.0/PI); //azimuthal angle
  int goal_e = floor(atan2(goal.z-pose.pose.position.z,sqrt((goal.x-pose.pose.position.x)*(goal.x-pose.pose.position.x)+(goal.y-pose.pose.position.y)*(goal.y-pose.pose.position.y)))*180.0/PI);//elevation angle

  waypt = getWaypointFromAngle(e,z);
  
  geometry_msgs::Point p;
  p.x = waypt.vector.x;
  p.y = waypt.vector.y;
  p.z = waypt.vector.z;

  if(velocity_x > 1.4 && braking_param ) {
    double obs_dist = distance2d(p,obs);
    double energy_waypoint = 0.5*1.56*velocity_x*velocity_x + 1.56*9.81*waypt.vector.z;
    double energy_threshold = 0.5*1.56*3*3 + 1.56*9.81*goal.z;
    double energy_diff = energy_waypoint-energy_threshold;
    double ke_diff =  0.5*1.56*velocity_x*velocity_x - 0.5*1.56*1.4*1.4;
    double pe_diff =  1.56*9.81*pose.pose.position.z - 1.56*9.81*goal.z;

    cost = (1/obs_dist)  + x_brake_cost_param*ke_diff*(waypt.vector.x-pose.pose.position.x) + z_brake_cost_param*pe_diff*(waypt.vector.z-pose.pose.position.z);

  }
  else {
  	if(velocity_x < 1.4)
  	first_brake = false;
  
  	if(e>-50) {
  		cost = goal_cost_param*sqrt((goal_e-e)*(goal_e-e)+(goal_z-z)*(goal_z-z)) + smooth_cost_param*sqrt((p1.x-e)*(p1.x-e)+(p1.y-z)*(p1.y-z)) ; //Best working cost function
  	}
  	else
  		cost = 10000;
  }

  return cost;  
}

void LocalPlanner::calculateCostMap() {

    int e = 0, z = 0;
    double cost_path; 
    float small ; int small_i;

    for(int i=0; i<path_candidates.cells.size(); i++) {  
        e = path_candidates.cells[i].x;
       	z = path_candidates.cells[i].y;
       	if(init == 0) {
       		p1.x = e;
       		p1.y = z;
      	}
      
       	cost_path = costFunction(e,z);
     
      	if(i == 0) {
        	small = cost_path;
        	small_i = i;
      	}

        if(cost_path<small) {
        	small = cost_path;
        	small_i = i ;
      	}  
    }
    
    p1.x = path_candidates.cells[small_i].x;
    p1.y = path_candidates.cells[small_i].y;
    p1.z = path_candidates.cells[small_i].z;
    path_selected.cells.push_back(p1);

   	ROS_INFO(" min_e - %f min_z- %f min_cost %f", p1.x,p1.y, small);

}

void LocalPlanner::getNextWaypoint() {

  	waypt = getWaypointFromAngle(p1.x,p1.y);
 
  	if(((waypt.vector.z<0.5) || checkForCollision()) && velocity_x<1.4 ) { 
    	waypt.vector.x = pose.pose.position.x-0.2;
    //waypt.vector.y = pose.y;
    	waypt.vector.z = pose.pose.position.z+ 0.2;
    	p1.x = 0;
    	p1.y = 0;
    	ROS_INFO(" Too close to the obstacle. Going back");
  	}
   	if(velocity_x>1.4 && pose.pose.position.z>0.5) { 
    
    //waypt.vector.x = stop_pose.x;
   	 	waypt.vector.y = 0; //stop_pose.y;global_path_pub
    //waypt.vector.z = stop_pose.z;
    	ROS_INFO(" Braking !! ");
    	p1.x = 0;
    	p1.y = 0;
  	}

    Eigen::Vector3d desired_position(waypt.vector.x, waypt.vector.y, waypt.vector.z);
    ROS_INFO("Publishing waypoint: [%f, %f, %f].",
           desired_position.x(),
           desired_position.y(),
           desired_position.z());

    last_waypt = waypt;

  //  publishWaypoint(waypt.vector.x, waypt.vector.y, waypt.vector.z);*/

 
}

bool LocalPlanner::checkForCollision() {
    bool avoid = false;
    geometry_msgs::Point temp;
    geometry_msgs::Point p;
    p.x = waypt.vector.x;
    p.y = waypt.vector.y;
    p.z = waypt.vector.z;

    pcl::PointCloud<pcl::PointXYZ>::iterator it;
    for( it = final_cloud.begin(); it != final_cloud.end(); ++it) {
      temp.x = it->x;
      temp.y = it->y;
      temp.z = it->z;
     
      if(distance2d(p,temp)< 0.5 && init != 0) { 
        printf("distance(p,temp)<0.5 \n");
        avoid = true;
        break;
      }
    }  
    return avoid;
}

void LocalPlanner::goFast()
{
   waypt.vector.x = waypt.vector.x + fast_waypoint_update_param;
   waypt.vector.y = goal.y;
   waypt.vector.z = goal.z;

   Eigen::Vector3d desired_position(waypt.vector.x, waypt.vector.y, waypt.vector.z);
  ROS_INFO("GO FAST Publishing waypoint: [%f, %f, %f].",
           desired_position.x(),
           desired_position.y(),
           desired_position.z());

    last_waypt = waypt;

}

void LocalPlanner::cropPointCloud() {  
  octomap::point3d half_min_cache;
  octomap::point3d half_max_cache;
  half_min_cache.x() = waypt.vector.x - min_cache_x;
  half_min_cache.y() = waypt.vector.y - min_cache_y;
  half_min_cache.z() = waypt.vector.z - min_cache_z;
  half_max_cache.x() = waypt.vector.x;
  half_max_cache.y() = waypt.vector.y + max_cache_y;
  half_max_cache.z() = waypt.vector.z + max_cache_z;

  octomapCloud.crop(half_min_cache, half_max_cache); 
}


geometry_msgs::PoseStamped LocalPlanner::createPoseMsg(geometry_msgs::Vector3Stamped waypt, double yaw) {
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.header.frame_id="/world";
  pose_msg.pose.position.x = waypt.vector.x;
  pose_msg.pose.position.y = waypt.vector.y;
  pose_msg.pose.position.z = waypt.vector.z;
  pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
  return pose_msg;
}

double LocalPlanner::nextYaw(geometry_msgs::Vector3Stamped u, geometry_msgs::Vector3Stamped v, double last_yaw) {
  int dx = v.vector.x - u.vector.x;
  int dy = v.vector.y - u.vector.y;
  if (dx == 0 && dy == 0) {
    return last_yaw;   // Going up or down
  }
  return atan2(dy, dx);
}

void LocalPlanner::getPathMsg() {

  path_msg.header.frame_id="/world";

  last_yaw = curr_yaw;
  geometry_msgs::Vector3Stamped curr_pose;
  curr_pose.vector.x = pose.pose.position.x;
  curr_pose.vector.y = pose.pose.position.y;
  curr_pose.vector.z = pose.pose.position.z;
  double new_yaw = nextYaw(last_waypt, waypt, last_yaw);

  waypt_p = createPoseMsg(waypt, new_yaw);

  path_msg.poses.push_back(waypt_p);

  curr_yaw = new_yaw;
} 

void LocalPlanner::goAhead() {

 // double yaw_1 = tf::getYaw(current_goal.pose.orientation);
 // double yaw_2 = tf::getYaw(pose.pose.orientation);

  last_goal = current_goal;

  double speed = 2.0;
  tf::Vector3 vec;
  vec.setX(current_goal.vector.x - pose.pose.position.x);
  vec.setY(current_goal.vector.y - pose.pose.position.y);
  vec.setZ(current_goal.vector.z - pose.pose.position.z);

  // = avoidance::toTfVector3(avoidance::subtractPoints(current_goal.vector, pose.pose.position));
    // If we are less than 1.0 away, then we should stop at the goal

  double new_len = vec.length() < 1.0 ? vec.length() : speed;
  vec.normalize();
  vec *= new_len;

//  setpoint = current_goal;  // The intermediate position sent to Mavros
  current_goal.vector.x = pose.pose.position.x + vec.getX();
  current_goal.vector.y = pose.pose.position.y + vec.getY();
  current_goal.vector.z = pose.pose.position.z + vec.getZ(); 

  publishWaypoint(pose.pose.position.x + vec.getX(), pose.pose.position.y + vec.getY(), pose.pose.position.z + vec.getZ());


  ROS_INFO("GO AHEAD publishing waypoint [%f, %f, %f].", setpoint.vector.x,  setpoint.vector.y,  setpoint.vector.z);
    // Publish setpoint for vizualization
//  current_waypoint_publisher_.publish(setpoint);

    // setpoint = rotatePoseMsgToMavros(setpoint); // 90 deg fix
 // listener_tf_.transformPose("local_origin", ros::Time(0), setpoint, "world", setpoint);

    // Publish setpoint to Mavros
//  mavros_waypoint_publisher_.publish(setpoint);  


} 





/*    waypt.vector.x = pose.pose.position.x+0.2;
    waypt.vector.y = pose.pose.position.y+0.2;
    waypt.vector.z = pose.pose.position.z;

    last_yaw = curr_yaw;
    geometry_msgs::Vector3Stamped curr_pose;
  curr_pose.vector.x = pose.pose.position.x;
  curr_pose.vector.y = pose.pose.position.y;
  curr_pose.vector.z = pose.pose.position.z;
    double new_yaw = nextYaw(last_waypt, waypt, last_yaw);
    path_msg.poses.push_back(createPoseMsg(waypt, new_yaw));
    last_waypt = waypt;


    Eigen::Vector3d desired_position(waypt.vector.x, waypt.vector.y, waypt.vector.z);
    ROS_INFO("GO AHEAD Publishing waypoint: [%f, %f, %f].",
           desired_position.x(),
           desired_position.y(),
           desired_position.z());
} */





void LocalPlanner::publishWaypoint(float x, float y, float z){

  setpoint.header.stamp = ros::Time::now();
  setpoint.header.frame_id = "/world";
  setpoint.vector.x = x;
  setpoint.vector.y = y;
  setpoint.vector.z = z;
}