//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#ifndef OCTOMAP_OMPL_RRT_H
#define OCTOMAP_OMPL_RRT_H

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <tf/transform_broadcaster.h>

#include <stdio.h>
#include <cstdlib>
#include <sstream>
#include <string>

#include <octomap/octomap.h>
#include <Eigen/Dense>

#include <global_planner/ompl_setup.h>

using namespace std;
using namespace Eigen;

class OctomapOmplRrt {
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ompl::OmplSetup problem_setup_;
  octomap::OcTree* map_;

  Eigen::Vector3d lower_bound_, upper_bound_;

 public:
  OctomapOmplRrt(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  virtual ~OctomapOmplRrt();

  void setupProblem();
  void setBounds(Eigen::Vector3d& lower_bound, Eigen::Vector3d& upper_bound);
  void setMap(octomap::OcTree* map);
  bool getPath(const Eigen::Vector3d& start, const Eigen::Vector3d& goal, std::vector<Eigen::Vector3d>* path);
  void solutionPathToTrajectoryPoints(ompl::geometric::PathGeometric& path,
                                      std::vector<Eigen::Vector3d>* trajectory_points) const;
};

#endif
