#include "local_planner.h"

LocalPlanner::LocalPlanner() {}

LocalPlanner::~LocalPlanner() {}

void LocalPlanner::setPose(const geometry_msgs::PoseStamped msg) {
  pose.header = msg.header;
  pose.pose.position = msg.pose.position;
  pose.pose.orientation = msg.pose.orientation;

  if (set_first_yaw){
    curr_yaw = tf::getYaw(msg.pose.orientation); 
  }

  setVelocity();  
  setLimitsBoundingBox(); 
}

void LocalPlanner::setVelocity() {

  velocity_x = curr_vel.twist.linear.x;
  velocity_y = curr_vel.twist.linear.y;
  velocity_z = curr_vel.twist.linear.z;
  velocity_mod = sqrt(pow(velocity_x,2) + pow(velocity_y,2) + pow(velocity_z,2));

}

void LocalPlanner::setLimitsBoundingBox() { 

  min_box.x = pose.pose.position.x - min_box_x;
  min_box.y = pose.pose.position.y - min_box_y;
  min_box.z = pose.pose.position.z - min_box_z;
  max_box.x = pose.pose.position.x + max_box_x;
  max_box.y = pose.pose.position.y + max_box_y;
  max_box.z = pose.pose.position.z + max_box_z;
  
}

void LocalPlanner::setGoal() {
  goal.x = goal_x_param;
  goal.y = goal_y_param;
  goal.z = goal_z_param;

  ROS_INFO("===== Set Goal ======: [%f, %f, %f].", goal.x, goal.y, goal.z);
}

void LocalPlanner::filterPointCloud(pcl::PointCloud<pcl::PointXYZ>& complete_cloud) {

  std::clock_t start_time = std::clock();
  pcl::PointCloud<pcl::PointXYZ>::iterator pcl_it;
  pcl::PointCloud<pcl::PointXYZ> front_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  final_cloud.points.clear();
  float min_distance = 1000;
  float distance;

  for (pcl_it = complete_cloud.begin(); pcl_it != complete_cloud.end(); ++pcl_it) {
      // Check if the point is invalid
    if (!std::isnan(pcl_it->x) && !std::isnan(pcl_it->y) && !std::isnan(pcl_it->z)) {
      if((pcl_it->x)<max_box.x && (pcl_it->x)>min_box.x && (pcl_it->y)<max_box.y && (pcl_it->y)>min_box.y && (pcl_it->z)<max_box.z && (pcl_it->z)>min_box.z) {
        cloud->points.push_back(pcl::PointXYZ(pcl_it->x, pcl_it->y, pcl_it->z));  
        final_cloud.push_back(pcl::PointXYZ(pcl_it->x, pcl_it->y, pcl_it->z)); 
        distance = sqrt(pow(pose.pose.position.x-pcl_it->x,2) + pow(pose.pose.position.y-pcl_it->y,2) + pow(pose.pose.position.z-pcl_it->z,2));
        if (distance < min_distance){
          min_distance = distance;
        } 
      }
    }
  }
  

  if(cloud->points.size() > 160 && (min_distance > pow(velocity_mod,2)/(2*deceleration_limit) + 0.5)) {    
    obstacle = true;
  }
  else {
    obstacle = false;
    ROS_INFO("False positive obstacle at distance %2.2fm. Minimum decelleration distance possible %2.2fm.", min_distance, pow(velocity_mod,2)/(2*deceleration_limit) + 0.5);
  }

  final_cloud.header.stamp =  complete_cloud.header.stamp;
  final_cloud.header.frame_id = complete_cloud.header.frame_id;
  final_cloud.width = final_cloud.points.size();
  final_cloud.height = 1; 


  ROS_INFO("Point cloud cropped im %2.2fms.", (std::clock() - start_time) / (double)(CLOCKS_PER_SEC / 1000));
  cloud_time.push_back((std::clock() - start_time) / (double)(CLOCKS_PER_SEC / 1000));

}

float distance3DCartesian(geometry_msgs::Point a, geometry_msgs::Point b) {

  return sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) + (a.z-b.z)*(a.z-b.z));
}

float distance2DPolar(int e1, int z1, int e2, int z2){
  return sqrt(pow((e1-e2),2) + pow((z1-z2),2));
}

bool LocalPlanner::obstacleAhead() { 
    
  if(obstacle){
    return true;
  } else{
    return false;
  }
}

void LocalPlanner::createPolarHistogram() {  

  std::clock_t start_time = std::clock();
  float bbx_rad = (max_box.x-min_box.x)*sqrt(2)/2; 
  float dist;
  geometry_msgs::Point temp; 
 
  polar_histogram.setZero();
  pcl::PointCloud<pcl::PointXYZ>::const_iterator it;

    
  for( it = final_cloud.begin(); it != final_cloud.end(); ++it) {   
    temp.x = it->x;
    temp.y = it->y;
    temp.z = it->z;
    dist = distance3DCartesian(pose.pose.position,temp);
   
    if(dist < bbx_rad) { 
      int beta_z = floor((atan2(temp.x-pose.pose.position.x,temp.y-pose.pose.position.y)*180.0/PI)); //(-180. +180]
      int beta_e = floor((atan((temp.z-pose.pose.position.z)/sqrt((temp.x-pose.pose.position.x)*(temp.x-pose.pose.position.x)+(temp.y-pose.pose.position.y)*(temp.y-pose.pose.position.y)))*180.0/PI)); //(-90.+90)

      beta_z = beta_z + (alpha_res - beta_z%alpha_res); //[-170,+190]
      beta_e = beta_e + (alpha_res - beta_e%alpha_res); //[-80,+90]
      
      int e = (90+beta_e)/alpha_res - 1; //[0,17]
      int z = (180+beta_z)/alpha_res - 1; //[0,35]

      polar_histogram.set(e,z,polar_histogram.get(e,z)+1);

    }
  }

  ROS_INFO("Polar histogram created in %2.2fms.", (std::clock() - start_time) / (double)(CLOCKS_PER_SEC / 1000));
  polar_time.push_back((std::clock() - start_time) / (double)(CLOCKS_PER_SEC / 1000));
}

void LocalPlanner::initGridCells(nav_msgs::GridCells *cell) {
  cell->cells.clear();
  cell->header.stamp = ros::Time::now();
  cell->header.frame_id = "/world";
  cell->cell_width = alpha_res;
  cell->cell_height = alpha_res;
}

void LocalPlanner::findFreeDirections() {

  std::clock_t start_time = std::clock();
  int n = floor(20/alpha_res); //safety radius
  int a = 0, b = 0;
  bool free = true;
  bool corner = false;
  geometry_msgs::Point p;
  geometry_msgs::Point Pp;
  std::vector<int> azimuthal_length(grid_length_z, 0);
  std::vector<int> elevation_length(grid_length_e, 0);
  std::vector<int> blocked_az(grid_length_z, 0);
  std::vector<int> blocked_el(grid_length_e, 0);
  std::vector<int> azimuthal_length_pos;
  extension_points.clear();

  initGridCells(&path_candidates);
  initGridCells(&path_rejected);
  initGridCells(&path_blocked);
  initGridCells(&path_selected);
  initGridCells(&path_extended);
  initGridCells(&Ppath_candidates);
  initGridCells(&Ppath_rejected);
  initGridCells(&Ppath_blocked);
  initGridCells(&Ppath_selected);

  for(int e= 0; e<grid_length_e; e++) {
    for(int z= 0; z<grid_length_z; z++) {  
      for(int i=e-n; i<=e+n; i++) {
        for(int j=z-n;j<=z+n;j++) {
                
          free = true;
          corner = false;

          //Case 1 - i < 0
          if(i<0 && j>=0 && j<grid_length_z) {
            a = -i;
            b = grid_length_z-j-1;
          }
          //Case 2 - j < 0
          else if(j<0 && i>=0 && i<grid_length_e) {
            b = j+grid_length_z;
          } 
          //Case 3 - i >= grid_length
          else if(i>=grid_length_e && j>=0 && j<grid_length_z) {
            a = grid_length_e-(i%(grid_length_e-1));
            b = grid_length_z-j-1;
          }
          //Case 4 - j >= grid_length
          else if(j>=grid_length_z && i>=0 && i<grid_length_e) {
            b = j%grid_length_z;
          }
          else if( i>=0 && i<grid_length_e && j>=0 && j<grid_length_z) {
            a = i;
            b = j;
          }
          else {
            corner = true;
          }

          if(!corner) {
            if(polar_histogram.get(a,b) != 0) {
              free = false;
              break;
            } 
          }
        }
            
        if(!free)
          break;
      }


      if(free) {    
        p.x = e*alpha_res+alpha_res-90; 
        p.y = z*alpha_res+alpha_res-180; 
        p.z = 0;                         
        path_candidates.cells.push_back(p);   
        publishPathCells(p.x, p.y, 0);      
      }
      else if(!free && polar_histogram.get(e,z) != 0) {
        azimuthal_length[z] = 1;
        elevation_length[e] = 1;
        p.x = e*alpha_res+alpha_res-90;  
        p.y = z*alpha_res+alpha_res-180; 
        p.z = 0;                         
        path_rejected.cells.push_back(p);  
        publishPathCells(p.x, p.y, 1);       
      }
      else {
        blocked_az[z] = 1;
        blocked_el[e] = 1;
        p.x = e*alpha_res+alpha_res-90;  
        p.y = z*alpha_res+alpha_res-180; 
        p.z = 0;                         
        path_blocked.cells.push_back(p); 
        publishPathCells(p.x, p.y, 2);               
      }
    } 
  }
   
  int e,z;
   
  if (pose.pose.position.z > 1.5){
    if (cv::countNonZero(azimuthal_length)>= 6){

      std::vector<int> idx_non_zeros;

      for (int c=0; c<azimuthal_length.size(); c++){
        if (azimuthal_length[c]==1)
          idx_non_zeros.push_back(c);
      }
      
      z = idx_non_zeros[floor(idx_non_zeros.size()/2)];
      int beta_z = z*alpha_res+alpha_res-180;

      int high_boundary = beta_z + 140;
      high_boundary = (high_boundary > 180) ? (-180 + (high_boundary%180))+180 : high_boundary+180;
      int low_boundary = beta_z - 130;
      low_boundary = (low_boundary < -170) ? (180 - std::abs(low_boundary)%180)+180 : low_boundary+180;

      if (high_boundary < low_boundary){
        int temp = low_boundary;
        low_boundary = high_boundary;
        high_boundary = temp;
      }
    //  printf("beta_z %d (%d) high %d (%d) low %d (%d) \n",beta_z+180, beta_z, high_boundary, high_boundary-180, low_boundary, low_boundary-180); 

      for (int i=0; i<grid_length_e; i++){
        if (elevation_length[i]==1){ //|| blocked_el[i]==1){
          e = i*alpha_res+alpha_res-90;
          
          for (int k=0; k<grid_length_z; k++){
            z = k*alpha_res+alpha_res-180;
            if ((beta_z+180) > low_boundary && (beta_z+180) < high_boundary){
              if (azimuthal_length[k]==0 && ((z+180)>low_boundary && (z+180)<=high_boundary) && (blocked_az[k]==0 || blocked_el[i]==0)) { // && (blocked_az[k]==0 || blocked_el[i]==0)){ 
                p.x = e; p.y = z; p.z = 0;
                path_blocked.cells.push_back(p);
                p = fromPolarToCartesian(e, z);
                path_extended.cells.push_back(p); 
              //  printf("caso 1 extended z %d \n", z+180);

                for(int t=0; t<path_candidates.cells.size(); t++){
                  if(path_candidates.cells[t].x==e && path_candidates.cells[t].y==z){
                    path_candidates.cells.erase(path_candidates.cells.begin()+t);
                  }
                }
              }
            }
            else {
              if (azimuthal_length[k]==0 && ((z+180)<low_boundary || (z+180)>=high_boundary) && (blocked_az[k]==0 || blocked_el[i]==0)) { // && (blocked_az[k]==0 || blocked_el[i]==0)){ 
                p.x = e; p.y = z; p.z = 0;
                path_blocked.cells.push_back(p);
                p = fromPolarToCartesian(e, z);
                path_extended.cells.push_back(p); 
               // printf("caso 2 extended z %d \n", z+180);

                for(int t=0; t<path_candidates.cells.size(); t++){
                  if(path_candidates.cells[t].x==e && path_candidates.cells[t].y==z){
                    path_candidates.cells.erase(path_candidates.cells.begin()+t);
                  }
                }
              }
            }
          } 
        }
      }
    }
   }
  ROS_INFO("Path candidates calculated in %2.2fms.",(std::clock() - start_time) / (double)(CLOCKS_PER_SEC / 1000));
  free_time.push_back((std::clock() - start_time) / (double)(CLOCKS_PER_SEC / 1000));
}

geometry_msgs::Point LocalPlanner::fromPolarToCartesian(int e, int z){

  geometry_msgs::Point p;
  p.x = pose.pose.position.x + rad*cos(e*(PI/180))*sin(z*(PI/180)); //round
  p.y = pose.pose.position.y + rad*cos(e*(PI/180))*cos(z*(PI/180));
  p.z = pose.pose.position.z + rad*sin(e*(PI/180));

  return p;

}

void LocalPlanner::publishPathCells(double e, double z, int path_type){

  geometry_msgs::Point p = fromPolarToCartesian((int)e, (int)z);

  if(path_type == 0){
    p.x = e; p.y = z; p.z=0;
    Ppath_candidates.cells.push_back(p);
  }
  if (path_type == 1){
    Ppath_rejected.cells.push_back(p); 
  }    
  if (path_type == 2){
    Ppath_blocked.cells.push_back(p); 
    
  }                      
}

geometry_msgs::Vector3Stamped LocalPlanner::getWaypointFromAngle(int e, int z) { 
  	
  geometry_msgs::Point p = fromPolarToCartesian(e, z);

  geometry_msgs::Vector3Stamped waypoint;
  waypoint.header.stamp = ros::Time::now();
  waypoint.header.frame_id = "/world";

  waypoint.vector.x = p.x;
  waypoint.vector.y = p.y;
  waypoint.vector.z = p.z;


  return waypoint;
}


double LocalPlanner::costFunction(int e, int z) {

  double cost;
  int goal_z = floor(atan2(goal.x-pose.pose.position.x,goal.y-pose.pose.position.y)*180.0/PI); //azimuthal angle
  int goal_e = floor(atan((goal.z-pose.pose.position.z)/sqrt((goal.x-pose.pose.position.x)*(goal.x-pose.pose.position.x)+(goal.y-pose.pose.position.y)*(goal.y-pose.pose.position.y)))*180.0/PI);//elevation angle
  int index;
  geometry_msgs::Vector3Stamped ww;
  ww = getWaypointFromAngle(e,z);
  
  geometry_msgs::Point p;
  p.x = ww.vector.x;
  p.y = ww.vector.y;
  p.z = ww.vector.z;
  
  double distance_cost = goal_cost_param * distance2DPolar(goal_e, goal_z, e, z);
  double smooth_cost = smooth_cost_param * distance2DPolar(p1.x, p1.y, e, z);
  double height_cost = std::abs(accumulated_height_prior[std::round(p.z)]-accumulated_height_prior[std::round(goal.z)])*prior_cost_param*10.0;
 // printf("%f %f %f \n", distance_cost, smooth_cost, height_cost);
 // int curr_z = floor(atan2(0,0)*180.0/PI); //azimuthal angle
//  int curr_e = floor(atan2(0,0)*180.0/PI);
//  double dist =  sqrt(pow(e-goal_e,2) + pow(z-goal_z,2)); //sqrt(pow(e-curr_e,2) + pow(z-curr_z,2));
//  double kinetic_energy = 0.5*1.56*std::abs(pow(velocity_x,2) + pow(velocity_y,2) + pow(velocity_z,2))*std::abs(dist);
//  double potential_energy = 1.56*9.81*std::abs(e-goal_e);
  cost = distance_cost + smooth_cost + height_cost;
  return cost;  
}

void LocalPlanner::calculateCostMap() {

  std::clock_t start_time = std::clock();
  int e = 0, z = 0;
  double cost_path; 
  float small ; int small_i;
  std::vector<float> cost_candidates(path_candidates.cells.size());

 // printf("p1.x %f p1.y %f \n", p1.x, p1.y);

  for(int i=0; i<path_candidates.cells.size(); i++) {  
    e = path_candidates.cells[i].x;
   	z = path_candidates.cells[i].y;
   	if(init == 0) {
   		p1.x = e;
   		p1.y = z;
   	}
      
   	cost_path = costFunction(e,z);
   // cost_candidates[i]=(cost_path);

   	if(i == 0) {
     	small = cost_path;
     	small_i = i;
   	}

    if(cost_path<small) {
     	small = cost_path;
     	small_i = i ;
   	}  
  }

//  cv::sortIdx(cost_candidates, cost_idx_sorted, CV_SORT_EVERY_ROW + CV_SORT_ASCENDING);
    
  p1.x = path_candidates.cells[small_i].x;
  p1.y = path_candidates.cells[small_i].y;
  p1.z = path_candidates.cells[small_i].z;
  path_selected.cells.push_back(p1);
    
  geometry_msgs::Point Pp1 = fromPolarToCartesian((int)p1.x, (int)p1.y);
  Ppath_selected.cells.push_back(Pp1);

  // for(int i=0; i<10; i++){
  //   e = path_candidates.cells[cost_idx_sorted[i]].x;
  //   z = path_candidates.cells[cost_idx_sorted[i]].y;
  //   printf("e %d z %d -> ",e,z);
  //   printf("%f \n", cost_candidates[cost_idx_sorted[i]]);
  // }

  ROS_INFO("Selected path (e, z) = (%d, %d) costs %.2f. Calculated in %2.2f ms.", (int)p1.x, (int)p1.y, small, (std::clock() - start_time) / (double)(CLOCKS_PER_SEC / 1000));
  cost_time.push_back((std::clock() - start_time) / (double)(CLOCKS_PER_SEC / 1000));

}

void LocalPlanner::getNextWaypoint() {
 
  setpoint = getWaypointFromAngle(p1.x,p1.y);
   
  if (withinGoalRadius() || !first_reach){
    ROS_INFO("Goal Reached: Hoovering");
    waypt.vector.x = waypt_stop.pose.position.x;
    waypt.vector.y = waypt_stop.pose.position.y;
    waypt.vector.z = waypt_stop.pose.position.z;
  }
  else{
    geometry_msgs::Vector3Stamped setpoint_temp, setpoint_temp2;
     
    tf::Vector3 vec;
    vec.setX(goal.x - pose.pose.position.x);
    vec.setY(goal.y - pose.pose.position.y);
    vec.setZ(goal.z - pose.pose.position.z);
    // double new_len = vec.length() < 1.0 ? vec.length() : speed;
    vec.normalize();
    vec *= 1.0;
      
 	  if(checkForCollision() && pose.pose.position.z>0.5) { //(velocity_x>1.4 && pose.pose.position.z>0.5) { 
      vec *= 5;
      waypt.vector.x = 0.0; //pose.pose.position.x - vec.getX(); 
      waypt.vector.y = 0.0; //pose.pose.position.y - vec.getY(); 
      waypt.vector.z = setpoint.vector.z;

   	  ROS_INFO("Braking!!! Obstacle closer than 0.5m.");
   	  p1.x = 0;
   	  p1.y = 0;
      path_selected.cells[path_selected.cells.size()-1].x = 0;
      path_selected.cells[path_selected.cells.size()-1].y = 0;
 	  }
    else{
      waypt = setpoint;
    }
  }

  ROS_INFO("Selected waypoint: [%f, %f, %f].", waypt.vector.x, waypt.vector.y, waypt.vector.z);
 
}


bool LocalPlanner::checkForCollision() {

  std::clock_t start_time = std::clock();
  bool avoid = false;
  geometry_msgs::Point temp;
  geometry_msgs::Point p, p_pose;
  p_pose.x = pose.pose.position.x;
  p_pose.y = pose.pose.position.y;
  p_pose.z = pose.pose.position.z;

  pcl::PointCloud<pcl::PointXYZ>::iterator it;
  for( it = final_cloud.begin(); it != final_cloud.end(); ++it) {
    temp.x = it->x;
    temp.y = it->y;
    temp.z = it->z;
      
    if(distance3DCartesian(p_pose,temp)< 0.5 && init != 0) { 
      avoid = true;
      break;
    }
  }  
  collision_time.push_back((std::clock() - start_time) / (double(CLOCKS_PER_SEC / 1000)));
 // printf("min dist %f min dist pose %f \n\n", min_dist, min_dist_pose_obst);
  return avoid;
}

void LocalPlanner::goFast(){

  if (withinGoalRadius() || !first_reach){

    ROS_INFO("Goal reached: hoovering.");
    waypt.vector.x = waypt_stop.pose.position.x;
    waypt.vector.y = waypt_stop.pose.position.y;
    waypt.vector.z = waypt_stop.pose.position.z;
  }
  else {
    tf::Vector3 vec;
    vec.setX(goal.x - pose.pose.position.x);
    vec.setY(goal.y - pose.pose.position.y);
    vec.setZ(goal.z - pose.pose.position.z);
    double new_len = vec.length() < 1.0 ? vec.length() : speed;
    vec.normalize();
    vec *= new_len;
  
    waypt.vector.x = pose.pose.position.x + vec.getX();
    waypt.vector.y = pose.pose.position.y + vec.getY();
    waypt.vector.z = pose.pose.position.z + vec.getZ();
 }

  
  ROS_INFO("Go fast selected waypoint: [%f, %f, %f].", waypt.vector.x, waypt.vector.y, waypt.vector.z);
}

bool LocalPlanner::withinGoalRadius(){

  geometry_msgs::Point a;
  a.x = std::abs(goal.x - pose.pose.position.x); 
  a.y = std::abs(goal.y - pose.pose.position.y);
  a.z = std::abs(goal.z - pose.pose.position.z);;
  
  if(a.x < 0.5 && a.y < 0.5 && a.z < 0.5){
    if (first_reach){
      waypt_stop = pose;
      first_reach = false;
    }
    return true;
  }
  else
    return false;

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
  double dx = v.vector.x - u.vector.x;
  double dy = v.vector.y - u.vector.y;

  if (round(dx) == 0 && round(dy) == 0) {
    return last_yaw;   // Going up or down
  }

  if (withinGoalRadius() || !first_reach){
    return last_yaw;
  }

  return atan2(dy, dx);
}

void LocalPlanner::reachGoalAltitudeFirst(){

  if (pose.pose.position.z < (goal_z_param - 0.5)) {
      waypt.vector.x = 0.0;
      waypt.vector.y = 0.0;
      waypt.vector.z = goal.z;
  } else {
    reach_altitude = true;
    printf("Reached altitude %f, now going towards the goal. \n\n",pose.pose.position.z);
    getPathMsg();
  }
}

void LocalPlanner::getPathMsg() {

  path_msg.header.frame_id="/world";
  last_waypt_p = waypt_p;
  last_yaw = curr_yaw;
  geometry_msgs::Vector3Stamped curr_pose;
  curr_pose.vector.x = pose.pose.position.x;
  curr_pose.vector.y = pose.pose.position.y;
  curr_pose.vector.z = pose.pose.position.z;

  //first reach the altitude of the goal then start to move towards it (optional, comment out the entire if) 
  if(!reach_altitude){
    reachGoalAltitudeFirst();
  }

  double new_yaw = nextYaw(curr_pose, waypt, last_yaw); 
  waypt_p = createPoseMsg(waypt, new_yaw);
  path_msg.poses.push_back(waypt_p);
  curr_yaw = new_yaw;
  checkSpeed();
} 

void LocalPlanner::checkSpeed(){

  if (hasSameYawAndAltitude(last_waypt_p, waypt_p) && !obstacleAhead()){
    speed = std::min(max_speed, speed + 0.1);
  }
  else{
    speed = min_speed;
  }

}

bool LocalPlanner::hasSameYawAndAltitude(geometry_msgs::PoseStamped msg1, geometry_msgs::PoseStamped msg2){

  return abs(msg1.pose.orientation.z) >= abs(0.9*msg2.pose.orientation.z) && abs(msg1.pose.orientation.z) <= abs(1.1*msg2.pose.orientation.z) 
         && abs(msg1.pose.orientation.w) >= abs(0.9*msg2.pose.orientation.w) && abs(msg1.pose.orientation.w) <= abs(1.1*msg2.pose.orientation.w)
         && abs(msg1.pose.position.z) >= abs(0.9*msg2.pose.position.z) && abs(msg1.pose.position.z) <= abs(1.1*msg2.pose.position.z);

}




