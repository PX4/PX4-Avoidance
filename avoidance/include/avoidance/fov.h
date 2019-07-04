#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>  //MatrixXi

#include "avoidance/common.h"

namespace avoidance {
/**
* Class defining the Field of View of the sensor. This is defined in a discrete
* manner in histogram resolution
*/
class FOV {
 public:
  /**
  * @brief     C'tor for a field of view
  * @param[in] number of cells of the histogram in horizontal direction
  * @param[in] number of cells of the histogram in vertical direction
  **/
  FOV(int yaw_num_cells, int pitch_num_cells);
  ~FOV() = default;

  /**
  * @brief     function to determine if a given direction in the fcu frame is
  *            within the field of view
  **/
  bool isVisible(float yaw_fcu_deg, float pitch_fcu_deg) const;

  /**
  * @brief       Update the field-of-view based on the given point
  * @param[in]   x coordinate of the point in the fcu frame
  * @param[in]   y coordinate of the point in the fcu frame
  * @param[in]   z coordinate of the point in the fcu frame
  **/
  void updateWithPoint(float x, float y, float z);

  /**
  * @brief       Compute a scale factor [0,1] that reflects how close to the
  *              edge of the field of view a given point is
  * @param[in]   yaw-orientation in the FCU frame of the point in question [deg]
  * @param[in]   pitch-orientation in the FCU frame of the point in question
  *[deg]
  **/
  float scaleToFOV(float yaw_fcu_deg, float pitch_fcu_deg) const;

 private:
  Eigen::MatrixXi is_visible_;

  int getYawIndex(float yaw_fcu_deg) const;
  int getPitchIndex(float pitch_fcu_deg) const;
};

}  // namespace avoidance
