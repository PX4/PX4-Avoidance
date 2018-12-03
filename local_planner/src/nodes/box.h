
#ifndef BOX_H
#define BOX_H

#include <geometry_msgs/Point.h>
namespace avoidance {
class Box {
 public:
  Box();
  Box(const double& radius);
  ~Box();

  void setBoxLimits(const geometry_msgs::Point& pos, const double ground_distance);
  bool isPointWithinBox(const double& x, const double& y, const double& z);

  double radius_;
  double box_dist_to_ground_ = 2.0;
  double zmin_;

 private:
  double xmin_;
  double xmax_;
  double ymin_;
  double ymax_;
  double zmax_;
};
}
#endif  // BOX_H
