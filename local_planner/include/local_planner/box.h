
#ifndef BOX_H
#define BOX_H

#include <geometry_msgs/Point.h>
namespace avoidance {
class Box {
 public:
  Box();
  Box(const float& radius);
  ~Box();

  void setBoxLimits(const geometry_msgs::Point& pos,
                    const float ground_distance);
  bool isPointWithinBox(const float& x, const float& y, const float& z);

  float radius_;
  float box_dist_to_ground_ = 2.0;
  float zmin_;

 private:
  float xmin_;
  float xmax_;
  float ymin_;
  float ymax_;
  float zmax_;
};
}
#endif  // BOX_H
