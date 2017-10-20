#include "local_planner.h"

LocalPlanner::LocalPlanner() {}

LocalPlanner::~LocalPlanner() {}

// update UAV pose
void LocalPlanner::setPose(const geometry_msgs::PoseStamped msg) {
  pose_.header = msg.header;
  pose_.pose.position = msg.pose.position;
  pose_.pose.orientation = msg.pose.orientation;

  if (set_first_yaw_){
    curr_yaw_ = tf::getYaw(msg.pose.orientation);
  }

  setVelocity();  
  setLimitsBoundingBox(); 
}

// update UAV velocity
void LocalPlanner::setVelocity() {
  velocity_x_ = curr_vel_.twist.linear.x;
  velocity_y_ = curr_vel_.twist.linear.y;
  velocity_z_ = curr_vel_.twist.linear.z;
  velocity_mod_ = sqrt(pow(velocity_x_,2) + pow(velocity_y_,2) + pow(velocity_z_,2));
}

// update bounding box limit coordinates around a new UAV pose
void LocalPlanner::setLimitsBoundingBox() { 
  min_box_.x = pose_.pose.position.x - min_box_x_;
  min_box_.y = pose_.pose.position.y - min_box_y_;
  min_box_.z = pose_.pose.position.z - min_box_z_;
  max_box_.x = pose_.pose.position.x + max_box_x_;
  max_box_.y = pose_.pose.position.y + max_box_y_;
  max_box_.z = pose_.pose.position.z + max_box_z_;
}

// set mission goal 
void LocalPlanner::setGoal() {
  goal_.x = goal_x_param_;
  goal_.y = goal_y_param_;
  goal_.z = goal_z_param_;
  reached_goal_ = false;
  ROS_INFO("===== Set Goal ======: [%f, %f, %f].", goal_.x, goal_.y, goal_.z);
  initGridCells(&path_waypoints_);
}

// trim the point cloud so that only points inside the bounding box are considered and
// filter out false positve obstacles
void LocalPlanner::filterPointCloud(pcl::PointCloud<pcl::PointXYZ>& complete_cloud) {
  std::clock_t start_time = std::clock();
  pcl::PointCloud<pcl::PointXYZ>::iterator pcl_it;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  final_cloud_.points.clear();
  min_distance_ = 1000.0f;
  float distance;
  double deceleration_limit = 1.5*9.8066;

  for (pcl_it = complete_cloud.begin(); pcl_it != complete_cloud.end(); ++pcl_it) {
      // Check if the point is invalid
    if (!std::isnan(pcl_it->x) && !std::isnan(pcl_it->y) && !std::isnan(pcl_it->z)) {
      if((pcl_it->x) < max_box_.x && (pcl_it->x) > min_box_.x && (pcl_it->y) < max_box_.y && (pcl_it->y) > min_box_.y && (pcl_it->z) < max_box_.z && (pcl_it->z) > min_box_.z) {
        cloud->points.push_back(pcl::PointXYZ(pcl_it->x, pcl_it->y, pcl_it->z));  
        distance = sqrt(pow(pose_.pose.position.x-pcl_it->x,2) + pow(pose_.pose.position.y-pcl_it->y,2) + pow(pose_.pose.position.z-pcl_it->z,2));
        if (distance < min_distance_){
          min_distance_ = distance;
        } 
      }
    }
  }

  // statistical outlier removal (really slow)
  if (cloud->points.size() > 0) {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK (20);
    sor.setStddevMulThresh (1.0);
    sor.filter(*cloud);

    // filter out obstacles that are smaller than 160 points and that are at a distance greater than
    // how much space the UAV needs to travel to stop in front of it when at maximum decelleration
    if(cloud->points.size() > 160 && (min_distance_ > pow(velocity_mod_,2)/(2*deceleration_limit) + 0.5)) {
      obstacle_ = true;
    }
    else {
      obstacle_ = false;
      ROS_INFO("False positive obstacle at distance %2.2fm. Minimum decelleration distance possible %2.2fm.", min_distance_, pow(velocity_mod_,2)/(2*deceleration_limit) + 0.5);
    }
  }

  final_cloud_.header.stamp =  complete_cloud.header.stamp;
  final_cloud_.header.frame_id = complete_cloud.header.frame_id;
  final_cloud_.width = cloud->points.size();
  final_cloud_.height = 1;
  final_cloud_.points = cloud->points;

  ROS_INFO("Point cloud cropped in %2.2fms. Cloud size %d.", (std::clock() - start_time) / (double)(CLOCKS_PER_SEC / 1000), final_cloud_.width);
  cloud_time_.push_back((std::clock() - start_time) / (double)(CLOCKS_PER_SEC / 1000));
}

float distance3DCartesian(geometry_msgs::Point a, geometry_msgs::Point b) {
  return sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) + (a.z-b.z)*(a.z-b.z));
}

float distance2DPolar(int e1, int z1, int e2, int z2){
  return sqrt(pow((e1-e2),2) + pow((z1-z2),2));
}

bool LocalPlanner::obstacleAhead() { 
  if(obstacle_){
    return true;
  } else{
    return false;
  }
}

// fill the 2D polar histogram with the points from the filtered point cloud
void LocalPlanner::createPolarHistogram() {  
  std::clock_t start_time = std::clock();
  float bbx_rad = (max_box_.x - min_box_.x) * sqrt(2.0f) / 2.0f;
  float dist;
  geometry_msgs::Point temp; 
 
  polar_histogram_.setZero();
  pcl::PointCloud<pcl::PointXYZ>::const_iterator it;

    
  for( it = final_cloud_.begin(); it != final_cloud_.end(); ++it) {
    temp.x = it->x;
    temp.y = it->y;
    temp.z = it->z;
    dist = distance3DCartesian(pose_.pose.position,temp);
   
    if(dist < bbx_rad) { 
      int beta_z = floor((atan2(temp.x-pose_.pose.position.x,temp.y-pose_.pose.position.y)*180.0/PI)); //(-180. +180]
      int beta_e = floor((atan((temp.z-pose_.pose.position.z)/sqrt((temp.x-pose_.pose.position.x)*(temp.x-pose_.pose.position.x)+(temp.y-pose_.pose.position.y)*(temp.y-pose_.pose.position.y)))*180.0/PI)); //(-90.+90)

      beta_z = beta_z + (alpha_res - beta_z%alpha_res); //[-170,+190]
      beta_e = beta_e + (alpha_res - beta_e%alpha_res); //[-80,+90]
      
      int e = (90+beta_e)/alpha_res - 1; //[0,17]
      int z = (180+beta_z)/alpha_res - 1; //[0,35]

      polar_histogram_.set(e,z,polar_histogram_.get(e,z)+1);

    }
  }

  ROS_INFO("Polar histogram created in %2.2fms.", (std::clock() - start_time) / (double)(CLOCKS_PER_SEC / 1000));
  polar_time_.push_back((std::clock() - start_time) / (double)(CLOCKS_PER_SEC / 1000));
}

// initialize GridCell message
void LocalPlanner::initGridCells(nav_msgs::GridCells *cell) {
  cell->cells.clear();
  cell->header.stamp = ros::Time::now();
  cell->header.frame_id = "/world";
  cell->cell_width = alpha_res;
  cell->cell_height = alpha_res;
  cell->cells = {};
}

// search for free directions in the 2D polar histogram with a moving window approach
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
  cost_path_candidates_.clear();

  initGridCells(&path_candidates_);
  initGridCells(&path_rejected_);
  initGridCells(&path_blocked_);
  initGridCells(&path_selected_);
  initGridCells(&path_extended_);

  for(int e = 0; e < grid_length_e; e++) {
    for(int z = 0; z < grid_length_z; z++) {  
      for(int i = (e-n); i <= (e+n); i++) {
        for(int j = (z-n); j <= (z+n); j++) {
                
          free = true;
          corner = false;

          // Elevation index < 0 
          if(i < 0 && j >= 0 && j < grid_length_z) {
            a = -i;
            b = grid_length_z - j - 1;
          }
          // Azimuth index < 0
          else if(j < 0 && i >= 0 && i < grid_length_e) {
            b = j+grid_length_z;
          } 
          // Elevation index > grid_length_e
          else if(i >= grid_length_e && j >= 0 && j < grid_length_z) {
            a = grid_length_e - (i % (grid_length_e - 1));
            b = grid_length_z - j - 1;
          }
          // Azimuth index > grid_length_z
          else if(j >= grid_length_z && i >= 0 && i < grid_length_e) {
            b = j % grid_length_z;
          }
          // Elevation and Azimuth index both within histogram
          else if( i >= 0 && i < grid_length_e && j >= 0 && j < grid_length_z) {
            a = i;
            b = j;
          }
          // Elevation and azimuth index both < 0 OR elevation index > grid_length_e and azimuth index <> grid_length_z
          // OR elevation index < 0 and azimuth index > grid_length_z OR elevation index > grid_length_e and azimuth index < 0.
          // These cells are not part of the polar histogram.
          else {
            corner = true;
          }

          if(!corner) {
            if(polar_histogram_.get(a,b) != 0) {
              free = false;
              break;
            } 
          }
        }
            
        if(!free)
          break;
      }

      if(free) {    
        p.x = e * alpha_res + alpha_res - 90;
        p.y = z * alpha_res + alpha_res - 180;
        p.z = 0;
        path_candidates_.cells.push_back(p);
        cost_path_candidates_.push_back(costFunction((int)p.x, (int)p.y));
      }
      else if(!free && polar_histogram_.get(e,z) != 0) {
        azimuthal_length[z] = 1;
        elevation_length[e] = 1;
        p.x = e * alpha_res + alpha_res - 90;
        p.y = z * alpha_res + alpha_res - 180;
        p.z = 0;
        path_rejected_.cells.push_back(p);
      }
      else {
        blocked_az[z] = 1;
        blocked_el[e] = 1;
        p.x = e * alpha_res + alpha_res - 90;
        p.y = z * alpha_res + alpha_res - 180;
        p.z = 0;
        path_blocked_.cells.push_back(p);
      }
    } 
  }
  
  // extend rejected path cells in the 2D polar histogram when the UAV altitude is greater than 1.5m and 
  // when the obstacle is longer than 60 degrees. The obstacle becomes 270 degree long, leaving only 90 degrees 
  // behind the UAV heading free
  int e,z;
  if (pose_.pose.position.z > 1.5){
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

      for (int i=0; i<grid_length_e; i++){
        if (elevation_length[i]==1){
          e = i*alpha_res+alpha_res-90;
          
          for (int k=0; k<grid_length_z; k++){
            z = k*alpha_res+alpha_res-180;
            if ((beta_z+180) > low_boundary && (beta_z+180) < high_boundary){
              if (azimuthal_length[k]==0 && ((z+180)>low_boundary && (z+180)<=high_boundary) && (blocked_az[k]==0 || blocked_el[i]==0)) {
                p.x = e; p.y = z; p.z = 0;
                path_blocked_.cells.push_back(p);
                path_extended_.cells.push_back(p);

                for(int t=0; t<path_candidates_.cells.size(); t++){
                  if(path_candidates_.cells[t].x==e && path_candidates_.cells[t].y==z){
                    path_candidates_.cells.erase(path_candidates_.cells.begin()+t);
                    cost_path_candidates_.erase(cost_path_candidates_.begin()+t);
                  }
                }
              }
            }
            else {
              if (azimuthal_length[k]==0 && ((z+180)<low_boundary || (z+180)>=high_boundary) && (blocked_az[k]==0 || blocked_el[i]==0)) {
                p.x = e; p.y = z; p.z = 0;
                path_blocked_.cells.push_back(p);
                path_extended_.cells.push_back(p);

                for(int t=0; t<path_candidates_.cells.size(); t++){
                  if(path_candidates_.cells[t].x==e && path_candidates_.cells[t].y==z){
                    path_candidates_.cells.erase(path_candidates_.cells.begin()+t);
                    cost_path_candidates_.erase(cost_path_candidates_.begin()+t);
                  }
                }
              }
            }
          } 
        }
      }
    }
   } else{
    for (int i = 0; i < 9; ++i) {
      for(int k=0; k<36 ; ++k){
        // fill entire lower half of the histogram
        p.x = i*alpha_res+alpha_res-90;
        p.y = k*alpha_res+alpha_res-180;
        p.z = 0.0f;
        path_blocked_.cells.push_back(p);

        for(int t=0; t<path_candidates_.cells.size(); t++){
          if(path_candidates_.cells[t].x==p.x && path_candidates_.cells[t].y==p.y){
            path_candidates_.cells.erase(path_candidates_.cells.begin()+t);
            cost_path_candidates_.erase(cost_path_candidates_.begin()+t);
          }
        }
      }
   }
 }

  ROS_INFO("Path candidates calculated in %2.2fms.",(std::clock() - start_time) / (double)(CLOCKS_PER_SEC / 1000));
  free_time_.push_back((std::clock() - start_time) / (double)(CLOCKS_PER_SEC / 1000));
}

// transform polar coordinates into Cartesian coordinates
geometry_msgs::Point LocalPlanner::fromPolarToCartesian(int e, int z){
  geometry_msgs::Point p;
  p.x = pose_.pose.position.x + rad_*cos(e*(PI/180))*sin(z*(PI/180)); //round
  p.y = pose_.pose.position.y + rad_*cos(e*(PI/180))*cos(z*(PI/180));
  p.z = pose_.pose.position.z + rad_*sin(e*(PI/180));

  return p;
}

// transform a 2D polar histogram direction in a 3D Catesian coordinate point
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

// cost function according to all free directions found in the 2D polar histogram are ranked
double LocalPlanner::costFunction(int e, int z) {
  double cost;
  int goal_z = floor(atan2(goal_.x - pose_.pose.position.x, goal_.y - pose_.pose.position.y)*180.0 / PI); //azimuthal angle
  int goal_e = floor(atan((goal_.z-pose_.pose.position.z) / sqrt(pow((goal_.x-pose_.pose.position.x), 2) + pow((goal_.y-pose_.pose.position.y), 2)))*180.0 / PI);//elevation angle
  geometry_msgs::Vector3Stamped possible_waypt = getWaypointFromAngle(e,z);
  
  double distance_cost = goal_cost_param_ * distance2DPolar(goal_e, goal_z, e, z);
  int waypoint_index = path_waypoints_.cells.size();
  double smooth_cost = smooth_cost_param_ * distance2DPolar(path_waypoints_.cells[waypoint_index-1].x, path_waypoints_.cells[waypoint_index-1].y, e, z);
  double height_cost = std::abs(accumulated_height_prior[std::round(possible_waypt.vector.z)]-accumulated_height_prior[std::round(goal_.z)])*prior_cost_param_*10.0;
  // alternative cost to prefer higher altitudes
  // int t = std::round(p.z) + e;
  // t = t < 0 ? 0 : t;
  // t = t > 21 ? 21 : t;
  // double height_cost = std::abs(accumulated_height_prior[t])*prior_cost_param*10.0;
  double kinetic_energy = 0.5*1.56*std::abs(pow(velocity_x_,2) + pow(velocity_y_,2) + pow(velocity_z_,2));
  double potential_energy = 1.56*9.81*std::abs(e-goal_e);
  cost = distance_cost + smooth_cost + height_cost;
  return cost;  
}


// calculate the free direction which has the smallest cost for the UAV to travel to
void LocalPlanner::calculateCostMap() {
  std::clock_t start_time = std::clock();
  cv::sortIdx(cost_path_candidates_, cost_idx_sorted_, CV_SORT_EVERY_ROW + CV_SORT_ASCENDING);
  
  geometry_msgs::Point p;
  p.x = path_candidates_.cells[cost_idx_sorted_[0]].x;
  p.y = path_candidates_.cells[cost_idx_sorted_[0]].y;
  p.z = path_candidates_.cells[cost_idx_sorted_[0]].z;
  path_selected_.cells.push_back(p);
  path_waypoints_.cells.push_back(p);

  ROS_INFO("Selected path (e, z) = (%d, %d) costs %.2f. Calculated in %2.2f ms.", (int)p.x, (int)p.y, cost_path_candidates_[cost_idx_sorted_[0]], (std::clock() - start_time) / (double)(CLOCKS_PER_SEC / 1000));
  cost_time_.push_back((std::clock() - start_time) / (double)(CLOCKS_PER_SEC / 1000));
}

// check that the selected direction is really free and transform it into a waypoint. Otherwise break not 
//to collide with an obstacle
void LocalPlanner::getNextWaypoint() {
  int waypoint_index = path_waypoints_.cells.size();
  geometry_msgs::Vector3Stamped setpoint = getWaypointFromAngle(path_waypoints_.cells[waypoint_index-1].x, path_waypoints_.cells[waypoint_index-1].y);

  if (withinGoalRadius()){
    ROS_INFO("Goal Reached: Hoovering");
    waypt_.vector.x = goal_.x;
    waypt_.vector.y = goal_.y;
    waypt_.vector.z = goal_.z;
  }
  else{
 	  if(checkForCollision() && pose_.pose.position.z>0.5) {
      waypt_.vector.x = 0.0;
      waypt_.vector.y = 0.0;
      waypt_.vector.z = setpoint.vector.z;

   	  ROS_INFO("Braking!!! Obstacle closer than 0.5m.");
      path_selected_.cells[path_selected_.cells.size()-1].x = 0;
      path_selected_.cells[path_selected_.cells.size()-1].y = 0;

      path_waypoints_.cells[path_waypoints_.cells.size()-1].x = 90;
      path_waypoints_.cells[path_waypoints_.cells.size()-1].y = 90;
 	  }
    else{
      waypt_ = setpoint;
    }
  }

  ROS_INFO("Selected waypoint: [%f, %f, %f].", waypt_.vector.x, waypt_.vector.y, waypt_.vector.z);
}

// check that each point in the filtered point cloud is at least 0.5m away from the waypoint the UAV is
// flying to
bool LocalPlanner::checkForCollision() {
  std::clock_t start_time = std::clock();
  bool avoid = false;
      
  if(min_distance_ < 0.5f && init != 0) {
    avoid = true;
  }
 
  collision_time_.push_back((std::clock() - start_time) / (double(CLOCKS_PER_SEC / 1000)));
  return avoid;
}

// if there isn't any obstacle in front of the UAV, increase cruising speed
void LocalPlanner::goFast(){

  if (withinGoalRadius()){
    ROS_INFO("Goal reached: hoovering.");
    waypt_.vector.x = goal_.x;
    waypt_.vector.y = goal_.y;
    waypt_.vector.z = goal_.z;
  }
  else {
    tf::Vector3 vec;
    vec.setX(goal_.x - pose_.pose.position.x);
    vec.setY(goal_.y - pose_.pose.position.y);
    vec.setZ(goal_.z - pose_.pose.position.z);
    double new_len = vec.length() < 1.0 ? vec.length() : speed;
    vec.normalize();
    vec *= new_len;
  
    waypt_.vector.x = pose_.pose.position.x + vec.getX();
    waypt_.vector.y = pose_.pose.position.y + vec.getY();
    waypt_.vector.z = pose_.pose.position.z + vec.getZ();

    // fill direction as straight ahead
    geometry_msgs::Point p; p.x = 90; p.y = 90; p.z = 0;
    path_waypoints_.cells.push_back(p);
 }

  
  ROS_INFO("Go fast selected waypoint: [%f, %f, %f].", waypt_.vector.x, waypt_.vector.y, waypt_.vector.z);
}

// check if the UAV has reached the goal set for the mission
bool LocalPlanner::withinGoalRadius(){
  geometry_msgs::Point a;
  a.x = std::abs(goal_.x - pose_.pose.position.x);
  a.y = std::abs(goal_.y - pose_.pose.position.y);
  a.z = std::abs(goal_.z - pose_.pose.position.z);
  float goal_acceptance_radius = 0.5f;
  
  if(a.x < goal_acceptance_radius && a.y < goal_acceptance_radius && a.z < goal_acceptance_radius){
    if(!reached_goal_){
      yaw_reached_goal_ = tf::getYaw(pose_.pose.orientation);
    }
    reached_goal_ = true;
    return true;
  }
  else
    return false;
}


geometry_msgs::PoseStamped LocalPlanner::createPoseMsg(geometry_msgs::Vector3Stamped waypt, double yaw) {
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.header.frame_id="/world";
  pose_msg.pose.position.x = waypt_.vector.x;
  pose_msg.pose.position.y = waypt_.vector.y;
  pose_msg.pose.position.z = waypt_.vector.z;
  pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
  return pose_msg;
}

// calculate the yaw for the next waypoint 
double LocalPlanner::nextYaw(geometry_msgs::PoseStamped u, geometry_msgs::Vector3Stamped v, double last_yaw_) {
  double dx = v.vector.x - u.pose.position.x;
  double dy = v.vector.y - u.pose.position.y;

  if (round(dx) == 0 && round(dy) == 0) {
    return last_yaw_;   // Going up or down
  }

  if (reached_goal_){
    return yaw_reached_goal_;
  }

  return atan2(dy, dx);
}

// when taking off, first publish waypoints to reach the goal altitude
void LocalPlanner::reachGoalAltitudeFirst(){
  if (pose_.pose.position.z < (goal_.z - 0.5)) {
      waypt_.vector.x = 0.0;
      waypt_.vector.y = 0.0;
      waypt_.vector.z = goal_.z;
  } else {
    reach_altitude_ = true;
    printf("Reached altitude %f, now going towards the goal_. \n\n",pose_.pose.position.z);
    getPathMsg();
  }
}

// smooth trajectory by liming the maximim accelleration possible
geometry_msgs::Vector3Stamped LocalPlanner::smoothWaypoint(){
  geometry_msgs::Vector3Stamped smooth_waypt;
  std::clock_t t = std::clock();
  float dt = (t - t_prev) / (float)(CLOCKS_PER_SEC);
  dt = dt > 0.0f ? dt : 0.004f;
  t_prev = t;
  Eigen::Vector2f vel_xy(curr_vel_.twist.linear.x, curr_vel_.twist.linear.y);
  Eigen::Vector2f vel_waypt_xy((waypt_.vector.x - last_waypt_p_.pose.position.x) / dt, (waypt_.vector.y - last_waypt_p_.pose.position.y) / dt);
  Eigen::Vector2f vel_waypt_xy_prev((last_waypt_p_.pose.position.x - last_last_waypt_p_.pose.position.x) / dt, (last_waypt_p_.pose.position.y - last_last_waypt_p_.pose.position.y) / dt);
  Eigen::Vector2f acc_waypt_xy((vel_waypt_xy - vel_waypt_xy_prev) / dt);

  if (acc_waypt_xy.norm() > (acc_waypt_xy.norm() / 2.0f)) {
    vel_xy = (acc_waypt_xy.norm() / 2.0f) * acc_waypt_xy.normalized() * dt + vel_waypt_xy_prev;
  }

  float vel_waypt_z = (waypt_.vector.z - last_waypt_p_.pose.position.z) / dt;
  float max_acc_z, vel_z;
  float vel_waypt_z_prev = (last_waypt_p_.pose.position.z - last_last_waypt_p_.pose.position.z) / dt;
  float acc_waypt_z = (vel_waypt_z - vel_waypt_z_prev) / dt;

  max_acc_z = (acc_waypt_z < 0.0f) ? -(max_accel_z_) : (max_accel_z_);
  if (fabsf(acc_waypt_z) > fabsf(max_acc_z)) {
    vel_z = max_acc_z * dt + vel_waypt_z_prev;
  }

  smooth_waypt.vector.x = last_waypt_p_.pose.position.x + vel_xy(0) * dt;
  smooth_waypt.vector.y = last_waypt_p_.pose.position.y + vel_xy(1) * dt;
  smooth_waypt.vector.z = waypt_.vector.z;

  ROS_INFO("Smothed waypoint: [%f %f %f].", smooth_waypt.vector.x, smooth_waypt.vector.y, smooth_waypt.vector.z);
  return smooth_waypt;
}

// create the message that is sent to the UAV
void LocalPlanner::getPathMsg() {
  path_msg_.header.frame_id="/world";
  last_last_waypt_p_ = last_waypt_p_;
  last_waypt_p_ = waypt_p_;
  last_yaw_ = curr_yaw_;

  //first reach the altitude of the goal then start to move towards it (optional, comment out the entire if) 
  if(!reach_altitude_){
    reachGoalAltitudeFirst();
  } else {
      if(!reached_goal_ && (pose_.pose.position.z > 1.5)){
        waypt_ = smoothWaypoint();
      }
  }

  double new_yaw = nextYaw(pose_, waypt_, last_yaw_);
  waypt_p_ = createPoseMsg(waypt_, new_yaw);
  path_msg_.poses.push_back(waypt_p_);
  curr_yaw_ = new_yaw;
  checkSpeed();
} 

void LocalPlanner::checkSpeed(){
  if (hasSameYawAndAltitude(last_waypt_p_, waypt_p_) && !obstacleAhead()){
    speed = std::min(max_speed_, speed + 0.1);
  }
  else{
    speed = min_speed_;
  }
}

// check if two points have the same altitude and yaw
bool LocalPlanner::hasSameYawAndAltitude(geometry_msgs::PoseStamped msg1, geometry_msgs::PoseStamped msg2){
  return abs(msg1.pose.orientation.z) >= abs(0.9*msg2.pose.orientation.z) && abs(msg1.pose.orientation.z) <= abs(1.1*msg2.pose.orientation.z) 
         && abs(msg1.pose.orientation.w) >= abs(0.9*msg2.pose.orientation.w) && abs(msg1.pose.orientation.w) <= abs(1.1*msg2.pose.orientation.w)
         && abs(msg1.pose.position.z) >= abs(0.9*msg2.pose.position.z) && abs(msg1.pose.position.z) <= abs(1.1*msg2.pose.position.z);

}

// old function to extend powerlines. TODO: check if there is anything useful
void LocalPlanner::extendPowerline(){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>); // need to take the filled one
  //estimation normal to the powerline
  if (pose_.pose.position.z > 1.5 && cloud->points.size()>3){
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud(cloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0) {
      PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    } 

    std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                    << coefficients->values[1] << " "
                                    << coefficients->values[2] << " "
                                    << coefficients->values[4] << std::endl;    
    coef1 = coefficients->values[0];  coef2 = coefficients->values[1];
    coef3 = coefficients->values[2];  coef4 = coefficients->values[3];
    
  }

  // extend powerlines in the polar histogram 
  std::vector<float> x_cloud, y_cloud, z_cloud;
  double minVal, maxVal;

  pcl::PointCloud<pcl::PointXYZ>::iterator pcl_it;
  for (pcl_it = final_cloud_.begin(); pcl_it != final_cloud_.end(); ++pcl_it) {
    x_cloud.push_back(pcl_it->x);
    y_cloud.push_back(pcl_it->y);
    z_cloud.push_back(pcl_it->z);
  }

  cv::meanStdDev(y_cloud, mean_y, stddev_y);
  cv::meanStdDev(z_cloud, mean_z, stddev_z);
  cv::meanStdDev(x_cloud, mean_x, stddev_x);
  printf("Mean Values [%f %f %f] \n", mean_x[0], mean_y[0], mean_z[0]);
  printf("Std Values [%f %f %f] \n", stddev_x[0], stddev_y[0], stddev_z[0]);

  cv::minMaxLoc(x_cloud, &minVal, &maxVal); printf("minx %f maxx %f \n", minVal, maxVal);
  cv::minMaxLoc(y_cloud, &minVal, &maxVal); printf("minx %f maxx %f \n", minVal, maxVal);
  cv::minMaxLoc(z_cloud, &minVal, &maxVal); printf("minx %f maxx %f \n", minVal, maxVal);
 
  ext_p1.x = mean_x[0]; ext_p2.x = mean_x[0];
  ext_p1.y = mean_y[0]-73.0; ext_p2.y = mean_y[0]+73.0;
  ext_p1.z = mean_z[0]; ext_p2.z = mean_z[0];

  ext_p1.x = (-coef2*ext_p1.y - coef3*ext_p1.z - coef4)/coef1;
  ext_p2.x = (-coef2*ext_p2.y - coef3*ext_p2.z - coef4)/coef1;
  extension_points.push_back(ext_p1);
  extension_points.push_back(ext_p2);
  int beta_z = floor((atan2(ext_p1.x-pose_.pose.position.x,ext_p1.y-pose_.pose.position.y)*180.0/PI)); //azimuthal angle
  int low_boundary = beta_z + (alpha_res - beta_z%alpha_res) + 180;
  beta_z = floor((atan2(ext_p2.x-pose_.pose.position.x,ext_p2.y-pose_.pose.position.y)*180.0/PI)); //azimuthal angle
  int high_boundary = beta_z + (alpha_res - beta_z%alpha_res) + 180;
  if (high_boundary < low_boundary){
    int temp = low_boundary;
    low_boundary = high_boundary;
    high_boundary = temp;
  }
}




