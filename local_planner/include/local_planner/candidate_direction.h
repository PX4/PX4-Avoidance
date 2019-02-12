#pragma once

namespace avoidance {

struct candidateDirection {
  float cost;
  float elevation_angle;
  float azimuth_angle;

  candidateDirection(float c, float e, float z)
      : cost(c), elevation_angle(e), azimuth_angle(z){};

  bool operator<(const candidateDirection& y) const { return cost < y.cost; }

  bool operator>(const candidateDirection& y) const { return cost > y.cost; }
};
}
