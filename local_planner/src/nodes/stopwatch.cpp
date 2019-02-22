#include "local_planner/stopwatch.h"

#include <ros/console.h>


namespace avoidance {
StopWatch::StopWatch(){
  start_time_ = ros::Time::now().toNSec();
};


void StopWatch::restart(){
	start_time_ = ros::Time::now().toNSec();
	split_time_ = start_time_;
	std::cout << "stopwatch start " << start_time_ << std::endl;
}

uint64_t StopWatch::elapsed(){
	uint64_t current_time = ros::Time::now().toNSec();
	duration_e_= current_time - start_time_;
	std::cout << "stopwatch current " << current_time << " duration "<< duration_e_ << std::endl;
	return duration_e_;
}

uint64_t StopWatch::split(){
	uint64_t last_time = split_time_;
	std::cout << "stopwatch2 last " << last_time << " split time "<< split_time_<< std::endl;
	split_time_ = ros::Time::now().toNSec();
	std::cout << "stopwatch2 last " << last_time << " split time "<< split_time_<< std::endl;
	duration_s_ = split_time_ - last_time;
	total_duration_ += duration_s_;
	return  duration_s_;
}

void StopWatch::setProcessTimeMsg(local_planner::ProcessTime& msg, const std::string& frame_id ){
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame_id;
  msg.function_name = function_name_;
  msg.duration = duration_e_;
  msg.counter = counter_;
}

}