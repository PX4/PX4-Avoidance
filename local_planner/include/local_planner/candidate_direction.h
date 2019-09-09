#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include "avoidance/common.h"
#include "trajectory_simulator.h"


namespace avoidance {

struct candidateDirection {
  float cost;
  float elevation_angle;
  float azimuth_angle;
  simulation_state trajectory_endpoint;

  candidateDirection(float c, float e, float z) : cost(c), elevation_angle(e), azimuth_angle(z){};

  bool operator<(const candidateDirection& y) const { return cost < y.cost; }

  bool operator>(const candidateDirection& y) const { return cost > y.cost; }

  PolarPoint toPolar(float r) const { return PolarPoint(elevation_angle, azimuth_angle, r); }
  Eigen::Vector3f toEigen() const {
    return Eigen::Vector3f(std::cos(elevation_angle * DEG_TO_RAD) * std::sin(azimuth_angle * DEG_TO_RAD),
                           std::cos(elevation_angle * DEG_TO_RAD) * std::cos(azimuth_angle * DEG_TO_RAD),
                           std::sin(elevation_angle * DEG_TO_RAD));
  }
};
}
