#ifndef OCTOMAP_OMPL_H
#define OCTOMAP_OMPL_H

#include <ompl/base/StateValidityChecker.h>
#include <octomap/octomap.h>

namespace ompl {

class OctomapValidityChecker : public base::StateValidityChecker {
 public:
  OctomapValidityChecker(const base::SpaceInformationPtr& space_info, octomap::OcTree* map)
      : base::StateValidityChecker(space_info), map_(map) {}
  virtual bool isValid(const base::State* state) const {

    Eigen::Vector3d position(state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0],
                              state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1],
                              state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2]);
    bool collision = checkCollision(position);
    if(collision) return false;
    else return true;
  }

  virtual bool checkCollision(Eigen::Vector3d state) const {

    bool collision = true;
    double occprob = 1.0;
    uint octree_depth = 16;
    double logodds;

    if(map_){
      octomap::OcTreeNode* node = map_->search(state(0), state(1), state(2), octree_depth);
      if (node)
        occprob = octomap::probability(logodds = node->getValue());
      else
        occprob = 0.5;  // Unobserved region of the map has equal chance of being occupied / unoccupied
  
    // Assuming a optimistic planner: Unknown space is considered as unoccupied
    if (occprob <= 0.5) collision = false;

    return collision;
    }
  }
 protected:
  octomap::OcTree* map_;
};
}  // namespace ompl

#endif
