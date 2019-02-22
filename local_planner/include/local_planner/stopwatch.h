#ifndef STOPWATCH_H
#define STOPWATCH_H
/**
* stopwatch functions to measure cpu process time
* reference:
* https://github.com/stonier/ecl_core/blob/release/0.61-indigo-kinetic/ecl_time/include/ecl/time/stopwatch.hpp
* run `sudo apt-get install ros-kinetic-ecl` before running the script
**/

#include <ros/time.h>
#include <queue>
#include <string>
#include <vector>

#include <local_planner/Profiling.h>

#include <local_planner/LocalPlannerNodeConfig.h>

namespace avoidance {
/**
 * @brief A timepiece that emulates the functionality of a stopwatch.
 *
 * This uses the TimeStamp class as a means of recording time, splits and
 * elapsed time. Its operation should be intuitive.
**/

class StopWatch {
 public:
  // store all durations of a function
  ros::Duration total_duration_;
  int counter_ = 0;
  std::string function_name_;

  ros::Publisher duration_measurement_pub_;
  /**
   * Default constructor that initialises the stopwatch with the current
   * system time to use as a point of reference.
   **/
  StopWatch();

  virtual ~StopWatch(){};

 private:
  ros::NodeHandle timing_node_;
};
void setProfilingMsg(local_planner::Profiling& msg, const std::string& frame_id,
                     const std::string& function_name, ros::Duration duration,
                     const int counter);
}
#endif  // STOPWATCH_H