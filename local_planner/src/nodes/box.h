
#ifndef BOX_H
#define BOX_H

#include <geometry_msgs/Point.h>

class Box {
 public:
  Box();
  Box(double radius);
  ~Box();

  void setBoxLimits(geometry_msgs::Point pos);
  bool isPointWithinBox(double x, double y, double z);


  double radius_;
  double zsize_up_;
  double zsize_down_;

 private:
  double xmin_;
  double xmax_;
  double ymin_;
  double ymax_;
  double zmin_;
  double zmax_;
};

#endif  // BOX_H
