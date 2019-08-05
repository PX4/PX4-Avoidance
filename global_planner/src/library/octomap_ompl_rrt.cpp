//  June/2019, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "global_planner/octomap_ompl_rrt.h"

using namespace Eigen;
using namespace std;
// Constructor
OctomapOmplRrt::OctomapOmplRrt(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {}
OctomapOmplRrt::~OctomapOmplRrt() {
  // Destructor
}

void OctomapOmplRrt::setupProblem() {
  problem_setup_.setDefaultPlanner();
  problem_setup_.setDefaultObjective();
  problem_setup_.setOctomapCollisionChecking(map_);
  ompl::base::RealVectorBounds bounds(3);
  bounds.setLow(0, lower_bound_.x());
  bounds.setLow(1, lower_bound_.y());
  bounds.setLow(2, lower_bound_.z());

  bounds.setHigh(0, upper_bound_.x());
  bounds.setHigh(1, upper_bound_.y());
  bounds.setHigh(2, upper_bound_.z());

  // Define start and goal positions.
  problem_setup_.getGeometricComponentStateSpace()->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);

  problem_setup_.setStateValidityCheckingResolution(0.001);
}

void OctomapOmplRrt::setBounds(Eigen::Vector3d& lower_bound, Eigen::Vector3d& upper_bound) {
  lower_bound_ = lower_bound;
  upper_bound_ = upper_bound;
}

bool OctomapOmplRrt::getPath(const Eigen::Vector3d& start, const Eigen::Vector3d& goal,
                             std::vector<Eigen::Vector3d>* path) {
  problem_setup_.clear();
  path->clear();

  ompl::base::ScopedState<ompl::base::RealVectorStateSpace> start_ompl(problem_setup_.getSpaceInformation());
  ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goal_ompl(problem_setup_.getSpaceInformation());

  start_ompl->values[0] = start(0);
  start_ompl->values[1] = start(1);
  start_ompl->values[2] = start(2);

  goal_ompl->values[0] = goal(0);
  goal_ompl->values[1] = goal(1);
  goal_ompl->values[2] = goal(2);
  problem_setup_.setStartAndGoalStates(start_ompl, goal_ompl);
  problem_setup_.setup();

  if (problem_setup_.solve(1.0)) {
    std::cout << "Found solution:" << std::endl;
    // problem_setup_.getSolutionPath().print(std::cout);
    // problem_setup_.simplifySolution();
    problem_setup_.getSolutionPath().print(std::cout);

  } else {
    std::cout << "Solution Not found" << std::endl;
  }

  if (problem_setup_.haveSolutionPath()) {
    solutionPathToTrajectoryPoints(problem_setup_.getSolutionPath(), path);
    return true;
  }
  return false;
}

void OctomapOmplRrt::setMap(octomap::OcTree* map) { map_ = map; }

void OctomapOmplRrt::solutionPathToTrajectoryPoints(ompl::geometric::PathGeometric& path,
                                                    std::vector<Eigen::Vector3d>* trajectory_points) const {
  // trajectory_points->clear();
  // trajectory_points->reserve(path.getStateCount());

  std::vector<ompl::base::State*>& state_vector = path.getStates();

  for (ompl::base::State* state_ptr : state_vector) {
    Eigen::Vector3d position(state_ptr->as<ompl::base::RealVectorStateSpace::StateType>()->values[0],
                             state_ptr->as<ompl::base::RealVectorStateSpace::StateType>()->values[1],
                             state_ptr->as<ompl::base::RealVectorStateSpace::StateType>()->values[2]);

    trajectory_points->emplace_back(position);
  }
}