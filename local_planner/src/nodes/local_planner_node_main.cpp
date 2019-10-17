#include <nodelet/loader.h>
#include <ros/ros.h>
#include <local_planner/local_planner_nodelet.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "local_planner_nodelet");

//   nodelet::Loader nodelet;
//   nodelet::M_string remap(ros::names::getRemappings());
//   nodelet::V_string nargv;
//   std::string nodelet_name = ros::this_node::getName();
//   nodelet.load(nodelet_name, "LocalPlannerNodelet", remap, nargv);
  avoidance::LocalPlannerNodelet nodelet;
  nodelet.onInit();
  ros::spin();

  return 0;
}
