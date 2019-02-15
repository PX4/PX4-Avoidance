
#ifndef BOX_H
#define BOX_H

#include <geometry_msgs/Point.h>
namespace avoidance {
class Box {
 public:
  Box();
  Box(const float& radius);
  ~Box() = default;

  /**
  * @brief     sets the bounding box coordinates limits around the vehicle
  *position
  * @param[in] pos, vehicle current position
  * @param[in] ground_distance, distance to the ground [m]
  **/
  void setBoxLimits(const geometry_msgs::Point& pos,
                    const float ground_distance);

  /**
  * @brief     checks if a pointcloud point is within the bounding box
  * @param[in] x, x-coordinate of the point
  * @param[in] y, y-coordinate of the point
  * @param[in] z, z-coordinate of the point
  * @returns   true, if the point is within the bounding box
  **/
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
