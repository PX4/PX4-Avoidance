#include "box.h"

Box::Box(const double& radius)
    : radius_{radius} {}

Box::Box()
    : xmin_{0.0}, xmax_{0.0}, ymin_{0.0}, ymax_{0.0}, zmin_{0.0},
	  zmax_{0.0}, radius_{0.0} {}

Box::~Box() {}

// update bounding box limit coordinates around a new UAV pose
void Box::setBoxLimits(const geometry_msgs::Point& pos, const double ground_distance) {
  double box_dist_to_ground = 0.5;  //distance of lowest part of the bounding box to the ground
  xmin_ = pos.x - radius_;
  ymin_ = pos.y - radius_;
  zmin_ = pos.z - ground_distance + box_dist_to_ground;
  xmax_ = pos.x + radius_;
  ymax_ = pos.y + radius_;
  zmax_ = pos.z + radius_;
}

bool Box::isPointWithinBox(const double& x, const double& y, const double& z) {
  return x < xmax_ && x > xmin_ && y < ymax_ && y > ymin_ && z < zmax_ &&
         z > zmin_;
}
