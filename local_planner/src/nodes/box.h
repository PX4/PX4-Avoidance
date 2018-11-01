
#ifndef BOX_H
#define BOX_H

#include <geometry_msgs/Point.h>

class Box {
 public:
  Box();
  Box(const double& radius);
  ~Box();

  void setBoxLimits(const geometry_msgs::Point& pos, const double ground_distance);
  bool isPointWithinBox(const double& x, const double& y, const double& z);


  double radius_;

 private:
  double xmin_;
  double xmax_;
  double ymin_;
  double ymax_;
  double zmin_;
  double zmax_;
};

#endif  // BOX_H
