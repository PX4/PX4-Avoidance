#include "local_planner/box.h"

namespace avoidance {

Box::Box(const float& radius)
    : xmin_{0.0f},
      xmax_{0.0f},
      ymin_{0.0f},
      ymax_{0.0f},
      zmin_{0.0f},
      zmax_{0.0f},
      radius_{radius},
      box_dist_to_ground_{1.0f} {}

Box::Box()
    : xmin_{0.0f},
      xmax_{0.0f},
      ymin_{0.0f},
      ymax_{0.0f},
      zmin_{0.0f},
      zmax_{0.0f},
      radius_{0.0f},
      box_dist_to_ground_{1.0f} {}

Box::~Box() {}

// update bounding box limit coordinates around a new UAV pose
void Box::setBoxLimits(const geometry_msgs::Point& pos,
                       const float ground_distance) {
  float zmin_close_to_ground = std::min(
      static_cast<float>(pos.z) + 0.8f,
      static_cast<float>(pos.z) - ground_distance + box_dist_to_ground_);
  zmin_ = std::max(zmin_close_to_ground, static_cast<float>(pos.z) - 1.0f);
  xmin_ = static_cast<float>(pos.x) - radius_;
  ymin_ = static_cast<float>(pos.y) - radius_;
  xmax_ = static_cast<float>(pos.x) + radius_;
  ymax_ = static_cast<float>(pos.y) + radius_;
  zmax_ = static_cast<float>(pos.z) + radius_;
}

bool Box::isPointWithinBox(const float& x, const float& y, const float& z) {
  return x < xmax_ && x > xmin_ && y < ymax_ && y > ymin_ && z < zmax_ &&
         z > zmin_;
}
}
