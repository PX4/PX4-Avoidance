#include "box.h"

Box::Box(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max)
    : xmin { x_min },
      xmax { x_max },
      ymin { y_min },
      ymax { y_max },
      zmin { z_min },
      zmax { z_max }{
}

Box::Box()
    : xmin { 0 },
      xmax { 0 },
      ymin { 0 },
      ymax { 0 },
      zmin { 0 },
      zmax { 0 } {
}

Box::~Box() {
}

// update bounding box limit coordinates around a new UAV pose
void Box::setLimitsHistogramBox(geometry_msgs::Point pos, Box size) {
  xmin = pos.x - size.xmin;
  ymin = pos.y - size.ymin;
  zmin = pos.z - size.zmin;
  xmax = pos.x + size.xmax;
  ymax = pos.y + size.ymax;
  zmax = pos.z + size.zmax;
}

// update bounding box limit coordinates around a new UAV pose
void Box::setLimitsGroundBox(geometry_msgs::Point pos, Box size, double min_dist) {
  xmin = pos.x - size.xmin;
  ymin = pos.y - size.ymin;
  zmin = pos.z - min_dist - size.zmin;
  xmax = pos.x + size.xmax;
  ymax = pos.y + size.ymax;
  zmax = pos.z;
}

