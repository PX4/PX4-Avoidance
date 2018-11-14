#include "box.h"

Box::Box(const double& radius) : radius_{radius} {}

Box::Box()
    : xmin_{0.0},
      xmax_{0.0},
      ymin_{0.0},
      ymax_{0.0},
      zmin_{0.0},
      zmax_{0.0},
      radius_{0.0},
      zsize_up_{0.0},
      zsize_down_{0.0} {}

Box::~Box() {}

// update bounding box limit coordinates around a new UAV pose
void Box::setBoxLimits(const geometry_msgs::Point& pos) {
  xmin_ = pos.x - radius_;
  ymin_ = pos.y - radius_;
  zmin_ = pos.z - zsize_down_;
  xmax_ = pos.x + radius_;
  ymax_ = pos.y + radius_;
  zmax_ = pos.z + zsize_up_;
}

bool Box::isPointWithinBox(const double& x, const double& y, const double& z) {
  return x < xmax_ && x > xmin_ && y < ymax_ && y > ymin_ && z < zmax_ &&
         z > zmin_;
}
