#pragma once

namespace avoidance {

struct candidateDirection {
  double cost;
  double elevation_angle;
  double azimuth_angle;

  candidateDirection(double c, double e, double z)
      : cost(c), elevation_angle(e), azimuth_angle(z){};

  bool operator<(const candidateDirection& y) const { return cost < y.cost; }

  bool operator>(const candidateDirection& y) const { return cost > y.cost; }
};
}
