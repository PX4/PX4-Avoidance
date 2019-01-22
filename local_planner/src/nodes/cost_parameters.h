#pragma once

namespace avoidance {

struct costParameters {
  double goal_cost_param;
  double smooth_cost_param;
  double height_change_cost_param = 4;
  double height_change_cost_param_adapted = 4;
};
}
