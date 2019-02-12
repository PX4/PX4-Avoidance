#pragma once

namespace avoidance {

struct costParameters {
  float goal_cost_param = 2.f;
  float smooth_cost_param = 1.5f;
  float height_change_cost_param = 4.f;
  float height_change_cost_param_adapted = 4.f;
};
}
