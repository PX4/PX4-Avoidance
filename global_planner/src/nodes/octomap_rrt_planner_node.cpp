//  June/2019, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "global_planner/octomap_rrt_planner.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "octomap_rrt_planner");
  ros::NodeHandle nh("~");
  ros::NodeHandle nh_private("");

  OctomapRrtPlanner* rrt_planner = new OctomapRrtPlanner(nh, nh_private);

  ros::spin();
  return 0;
}
