#pragma once

namespace avoidance {

struct costParameters {
  float heading_cost_param = 0.5f;
  float goal_cost_param = 3.f;
  float smooth_cost_param = 1.5f;
  float height_change_cost_param = 4.f;
  float height_change_cost_param_adapted = 4.f;
};
}
