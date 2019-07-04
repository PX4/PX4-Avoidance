#pragma once

#include <float.h>
#include <math.h>
#include <tf/transform_listener.h>
#include <deque>
#include <mutex>
#include <string>
#include <unordered_map>
#include <fstream>

namespace avoidance {

class TransformBuffer {
  std::unordered_map<std::string, std::deque<tf::StampedTransform>> buffer_;
  std::unique_ptr<std::mutex> mutex_;
  ros::Duration buffer_size_ = ros::Duration(10);

  std::string get_key(const std::string& source_frame,
                      const std::string& target_frame) const {
    return source_frame + "_to_" + target_frame;
  }

 public:
  std::vector<std::pair<std::string, std::string>> registered_transforms_;
  TransformBuffer(float buffer_size_s)
      : buffer_size_(ros::Duration(buffer_size_s)) {
    mutex_.reset(new std::mutex);
  };
  ~TransformBuffer() = default;

  /**
  * @brief     initialized a new deque in the map
  * @param[in] source_frame
  * @param[in] target_frame
  **/
  inline void registerTransform(const std::string& source_frame,
                                const std::string& target_frame) {
    std::lock_guard<std::mutex> lck(*mutex_);
    if (!isRegistered(source_frame, target_frame)) {
      std::deque<tf::StampedTransform> empty_deque;
      std::pair<std::string, std::string> transform_frames;
      transform_frames.first = source_frame;
      transform_frames.second = target_frame;
      registered_transforms_.push_back(transform_frames);
      buffer_[get_key(source_frame, target_frame)] = empty_deque;
      ROS_WARN("transform buffer: Registered %s",
               get_key(source_frame, target_frame).c_str());
    }
  }

  /**
  * @brief     return whether a transform is registered with the buffer
  * @param[in] source_frame
  * @param[in] target_frame
  * @param[in] bool, true if registered
  **/
  inline bool isRegistered(const std::string& source_frame,
                           const std::string& target_frame) {
    std::unordered_map<std::string,
                       std::deque<tf::StampedTransform>>::const_iterator
        iterator = buffer_.find(get_key(source_frame, target_frame));
    if (iterator == buffer_.end()) {
      return false;
    } else {
      return true;
    }
  }

  /**
  * @brief     inserts transform into buffer
  * @param[in] source_frame
  * @param[in] target_frame
  * @param[in] transform
  **/
  inline void insertTransform(const std::string& source_frame,
                              const std::string& target_frame,
                              tf::StampedTransform transform) {
    std::lock_guard<std::mutex> lck(*mutex_);
    std::unordered_map<std::string, std::deque<tf::StampedTransform>>::iterator
        iterator = buffer_.find(get_key(source_frame, target_frame));
    if (iterator == buffer_.end()) {
      ROS_ERROR("TF cannot be written to buffer, unregistered transform");
    } else {
      iterator->second.push_back(transform);
      while (transform.stamp_ - iterator->second.front().stamp_ >
             buffer_size_) {
        iterator->second.pop_front();
      }
    }
  }

  /**
  * @brief     retrieves transform from buffer
  * @param[in] source_frame
  * @param[in] target_frame
  * @param[in] time, time of requested tf
  * @param[out] transform
  **/
  inline bool getTransform(const std::string& source_frame,
                           const std::string& target_frame, ros::Time time,
                           tf::StampedTransform& transform) {
    std::lock_guard<std::mutex> lck(*mutex_);
    std::unordered_map<std::string,
                       std::deque<tf::StampedTransform>>::const_iterator
        iterator = buffer_.find(get_key(source_frame, target_frame));
    if (iterator == buffer_.end()) {
      ROS_ERROR(
          "could not retrieve requested transform from buffer, unregistered");
      return false;
    } else if (iterator->second.size() == 0) {
      ROS_WARN(
          "could not retrieve requested transform from buffer, buffer is "
          "empty");
      return false;
    } else {
      if (iterator->second.back().stamp_ < time) {
        ROS_WARN(
            "could not retrieve requested transform from buffer, tf has not "
            "yet arrived");
        return false;
      } else if (iterator->second.front().stamp_ > time) {
        ROS_WARN(
            "could not retrieve requested transform from buffer, tf has "
            "already been dropped from buffer");
        return false;
      } else {
        ros::Time last_sample_time = iterator->second.back().stamp_;
        for (std::deque<tf::StampedTransform>::const_reverse_iterator it =
                 iterator->second.rbegin();
             it != iterator->second.rend(); ++it) {
          if (it->stamp_ <= time) {
            ros::Duration dt = time - it->stamp_;
            ros::Duration sample_time = last_sample_time - it->stamp_;

            if (dt.toNSec() <= 0.5 * sample_time.toNSec()) {
              transform = *it;
              std::ofstream myfile1("/data/tf_delay", std::ofstream::app);
              myfile1 << dt.toNSec()<<"\t"<< it->stamp_<<"\t"<<time <<"\t"<<1<< "\n";
              myfile1.close();
              return true;
            } else {
              it--;
              transform = *it;
              std::ofstream myfile1("/data/tf_delay", std::ofstream::app);
              myfile1 << (it->stamp_ - time).toNSec()<<"\t"<< it->stamp_<<"\t"<<time <<"\t"<<2<< "\n";
              myfile1.close();
              return true;
            }
          }
          last_sample_time = it->stamp_;
        }
      }
    }
    return false;
  }
};
}
