#ifndef OMPL_SETUP_H
#define OMPL_SETUP_H

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include "ompl/base/SpaceInformation.h"

#include <global_planner/ompl_octomap.h>

namespace ompl {

class OmplSetup : public geometric::SimpleSetup {
 public:
  OmplSetup() : geometric::SimpleSetup(base::StateSpacePtr(new base::RealVectorStateSpace(3))) {}

  void setDefaultObjective() {
    getProblemDefinition()->setOptimizationObjective(
        ompl::base::OptimizationObjectivePtr(new ompl::base::PathLengthOptimizationObjective(getSpaceInformation())));
  }

  void setDefaultPlanner() { setRrtStar(); }

  void setRrtStar() { setPlanner(ompl::base::PlannerPtr(new ompl::geometric::RRTstar(getSpaceInformation()))); }

  const base::StateSpacePtr& getGeometricComponentStateSpace() const { return getStateSpace(); }

  void setStateValidityCheckingResolution(double resolution) {
    // This is a protected attribute, so need to wrap this function.
    si_->setStateValidityCheckingResolution(resolution);
  }

  void setOctomapCollisionChecking(octomap::OcTree* map) {
    std::shared_ptr<OctomapValidityChecker> validity_checker(new OctomapValidityChecker(getSpaceInformation(), map));

    setStateValidityChecker(base::StateValidityCheckerPtr(validity_checker));
  }
};

}  // namespace ompl

#endif
