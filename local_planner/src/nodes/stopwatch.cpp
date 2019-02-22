#include "local_planner/stopwatch.h"

#include <ros/console.h>

namespace avoidance {

StopWatch::StopWatch() {
  timing_node_ = ros::NodeHandle("~");
  duration_measurement_pub_ =
      timing_node_.advertise<local_planner::Profiling>("/performance_check", 1);
}

void setProfilingMsg(local_planner::Profiling& msg, const std::string& frame_id,
                     const std::string& function_name, ros::Duration duration,
                     const int counter) {
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame_id;
  msg.function_name = function_name;
  msg.duration = duration;
  msg.counter = counter;
}
}
