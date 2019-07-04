#pragma once

#include <float.h>
#include <math.h>
#include <tf/transform_listener.h>
#include <deque>
#include <mutex>
#include <string>
#include <unordered_map>

namespace avoidance {

class TransformBuffer {

protected:
  std::unordered_map<std::string, std::deque<tf::StampedTransform>> buffer_;
  std::unique_ptr<std::mutex> mutex_;
  ros::Duration buffer_size_;

  std::string get_key(const std::string& source_frame,
                      const std::string& target_frame) const {
    return source_frame + "_to_" + target_frame;
  }

  /**
  * @brief      interpolates between transforms
  * @param[in]  tf_earlier
  * @param[in]  tf_later
  * @param[out] transform, empty transform containing the desired timestep
  **/
  inline bool interpolateTransform(const tf::StampedTransform& tf_earlier,
                                   const tf::StampedTransform& tf_later,
                                   tf::StampedTransform& transform) {
    if (transform.stamp_ > tf_later.stamp_ ||
        transform.stamp_ < tf_earlier.stamp_) {
      return false;
    }

    ros::Duration timeBetween = tf_later.stamp_ - tf_earlier.stamp_;
    ros::Duration timeAfterEarlier = transform.stamp_ - tf_earlier.stamp_;
    float tau =
        static_cast<float>(timeAfterEarlier.toNSec()) / timeBetween.toNSec();

    tf::Vector3 translation =
        tf_earlier.getOrigin() * (1 - tau) + tf_later.getOrigin() * tau;
    tf::Quaternion rotation =
        tf_earlier.getRotation().slerp(tf_later.getRotation(), tau);

    transform.setOrigin(translation);
    transform.setRotation(rotation);
    return true;
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



 public:
  std::vector<std::pair<std::string, std::string>> registered_transforms_;

  TransformBuffer(float buffer_size_s = 10.0f)
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

      // check if this is a new transform
      bool new_tf = false;
      if(iterator->second.size() == 0){
    	  new_tf = true;
      }else if(iterator->second.back().stamp_ != transform.stamp_){
    	  new_tf = true;
      }

      if (new_tf) {
        iterator->second.push_back(transform);
        while (transform.stamp_ - iterator->second.front().stamp_ >
               buffer_size_) {
          iterator->second.pop_front();
        }
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
        for (std::deque<tf::StampedTransform>::const_reverse_iterator it =
                 iterator->second.rbegin();
             it != iterator->second.rend(); ++it) {
          if (it->stamp_ <= time) {
            const tf::StampedTransform& tf_earlier = *it;
            it--;
            const tf::StampedTransform& tf_later = *it;
            transform.stamp_ = time;
            if (interpolateTransform(tf_earlier, tf_later, transform)) {
              return true;
            } else {
              ROS_WARN("could not interpolate transform");
              return false;
            }
          }
        }
      }
    }
    return false;
  }
};
}
