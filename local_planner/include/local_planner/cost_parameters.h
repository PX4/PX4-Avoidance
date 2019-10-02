#pragma once

namespace avoidance {

struct costParameters {
  float yaw_cost_param = 0.5f;
  float pitch_cost_param = 3.f;
  float velocity_cost_param = 1.5f;
  float obstacle_cost_param = 5.0f;
  float distance_weigth_cost_param = 1000.f;
  float tree_hysteresis_param = 2500.f;
};
}
