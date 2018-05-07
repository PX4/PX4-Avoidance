#include "box.h"

Box::Box(double x_min, double x_max, double y_min, double y_max, double z_min,
         double z_max)
    : xmin_{x_min},
      xmax_{x_max},
      ymin_{y_min},
      ymax_{y_max},
      zmin_{z_min},
      zmax_{z_max} {}

Box::Box()
    : xmin_{0.0}, xmax_{0.0}, ymin_{0.0}, ymax_{0.0}, zmin_{0.0}, zmax_{0.0} {}

Box::~Box() {}

// update bounding box limit coordinates around a new UAV pose
void Box::setLimitsHistogramBox(geometry_msgs::Point pos, Box size) {
  xmin_ = pos.x - size.xmin_;
  ymin_ = pos.y - size.ymin_;
  zmin_ = pos.z - size.zmin_;
  xmax_ = pos.x + size.xmax_;
  ymax_ = pos.y + size.ymax_;
  zmax_ = pos.z + size.zmax_;
}

// update bounding box limit coordinates around a new UAV pose
void Box::setLimitsGroundBox(geometry_msgs::Point pos, Box size,
                             double min_dist) {
  xmin_ = pos.x - size.xmin_;
  ymin_ = pos.y - size.ymin_;
  zmin_ = pos.z - min_dist - size.zmin_;
  xmax_ = pos.x + size.xmax_;
  ymax_ = pos.y + size.ymax_;
  zmax_ = pos.z;
}

bool Box::isPointWithin(double x, double y, double z) {
  return x < xmax_ && x > xmin_ && y < ymax_ && y > ymin_ && z < zmax_ &&
         z > zmin_;
}
