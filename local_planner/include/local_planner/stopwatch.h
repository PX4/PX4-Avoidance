#ifndef STOPWATCH_H
#define STOPWATCH_H
/**
* stopwatch functions to measure cpu process time 
* reference: https://github.com/stonier/ecl_core/blob/release/0.61-indigo-kinetic/ecl_time/include/ecl/time/stopwatch.hpp
* 
**/

#include <ros/time.h>
#include <string>
#include <queue>
#include <vector>

#include <local_planner/ProcessTime.h>


namespace avoidance {
/**
 * @brief A timepiece that emulates the functionality of a stopwatch.
 *
 * This uses the TimeStamp class as a means of recording time, splits and
 * elapsed time. Its operation should be intuitive.
**/

class StopWatch{
public:
	// store all durations of a function 
    std::vector<uint64_t> timings_;
    uint64_t total_duration_;
    uint64_t duration_e_;
    uint64_t duration_s_;
    int counter_=0;
    std::string function_name_; 
    /**
     * Default constructor that initialises the stopwatch with the current
     * system time to use as a point of reference.
     **/	
	StopWatch();
    
    virtual ~StopWatch() {}

    /**
    * @brief restarts the stopwatch
    **/
    void restart();
    
    /**
    * @brief Calculates the total elapsed time
    * @return ros::Duration: the total elapsed time since (re)started. 
    **/
    uint64_t elapsed();
    /**
     * @brief Calculates the current split.
     *
     * Calculates the elapsed time since the last split.
     * @return ros::Duration : the elapsed time since the last split.
     **/
    uint64_t split();
    /**
     * @brief Create ProcessTIme message
     **/
    void setProcessTimeMsg(local_planner::ProcessTime& msg,const std::string& frame_id );

private:
	uint64_t start_time_, split_time_;


};

}
#endif  // STOPWATCH_H