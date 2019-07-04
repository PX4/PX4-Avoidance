#include "avoidance/fov.h"
#include "avoidance/common.h"

namespace avoidance {

FOV::FOV(int yaw_num_cells, int pitch_num_cells)
    : is_visible_(yaw_num_cells, pitch_num_cells) {
  is_visible_.fill(0);
}

bool FOV::isVisible(float yaw_fcu_deg, float pitch_fcu_deg) const {
  return is_visible_(getYawIndex(yaw_fcu_deg), getPitchIndex(pitch_fcu_deg));
}

void FOV::updateWithPoint(float x, float y, float z) {
  PolarPoint p = cartesianToPolarFCU(x, y, z);
  is_visible_(getYawIndex(p.z), getPitchIndex(p.e)) = 1;
}

float FOV::scaleToFOV(float yaw_fcu_deg, float pitch_fcu_deg) const {
  return 1.0f;
}

int FOV::getYawIndex(float yaw_fcu_deg) const {
  return static_cast<int>((yaw_fcu_deg + 180.0f) / 360.0f *
                          is_visible_.rows()) %
         is_visible_.rows();
}

int FOV::getPitchIndex(float pitch_fcu_deg) const {
  return static_cast<int>((pitch_fcu_deg + 90.0f) / 180.0f *
                          is_visible_.cols()) %
         is_visible_.cols();
}

}  // namespace avoidance
