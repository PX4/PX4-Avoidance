#pragma once
#include "avoidance/common.h"

namespace avoidance {

struct candidateDirection {
  float total_cost;
  float distance_cost;
  float velocity_cost;
  float yaw_cost;
  float pitch_cost;
  float elevation_angle;
  float azimuth_angle;

  candidateDirection(float c, float d, float v, float y, float p, float e, float z)
      : total_cost(c),
        velocity_cost(v),
        yaw_cost(y),
        pitch_cost(p),
        distance_cost(d),
        elevation_angle(e),
        azimuth_angle(z){};

  bool operator<(const candidateDirection& y) const { return total_cost < y.total_cost; }

  bool operator>(const candidateDirection& y) const { return total_cost > y.total_cost; }

  PolarPoint toPolar(float r) const { return PolarPoint(elevation_angle, azimuth_angle, r); }
};
}
