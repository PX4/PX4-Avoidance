#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include "avoidance/common.h"
#include "local_planner/tree_node.h"


namespace avoidance {

struct candidateDirection {
  float cost;
  float elevation_angle;
  float azimuth_angle;
  TreeNode tree_node;

  candidateDirection(float c, float e, float z) : cost(c), elevation_angle(e), azimuth_angle(z) {
    simulation_state start_state;
    start_state.position = Eigen::Vector3f(0.0f, 0.0f, 0.0f);;
    start_state.velocity = Eigen::Vector3f(0.0f, 0.0f, 0.0f);;
    start_state.acceleration = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    start_state.time = ros::Time::now().toSec();
    tree_node = TreeNode(0, start_state, Eigen::Vector3f::Zero(), 0.f);
  };

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
