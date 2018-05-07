
#ifndef BOX_H
#define BOX_H

#include <geometry_msgs/Point.h>

class Box {
 public:
  Box();
  Box(double x_min, double x_max, double y_min, double y_max, double z_min,
      double z_max);
  ~Box();

  void setLimitsHistogramBox(geometry_msgs::Point pos, Box size);
  void setLimitsGroundBox(geometry_msgs::Point pos, Box size, double min_dist);
  bool isPointWithin(double x, double y, double z);

  double xmin_;
  double xmax_;
  double ymin_;
  double ymax_;
  double zmin_;
  double zmax_;
};

#endif  // BOX_H
