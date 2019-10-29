#include "avoidance/transform_buffer.h"

namespace avoidance {

namespace tf_buffer {

TransformBuffer::TransformBuffer(float buffer_size_s) : buffer_size_(ros::Duration(buffer_size_s)) {
  startup_time_ = ros::Time::now();
};

std::string TransformBuffer::getKey(const std::string& source_frame, const std::string& target_frame) const {
  return source_frame + "_to_" + target_frame;
}

bool TransformBuffer::interpolateTransform(const tf::StampedTransform& tf_earlier, const tf::StampedTransform& tf_later,
                                           tf::StampedTransform& transform) const {
  // check if the requested timestamp lies between the two given transforms
  if (transform.stamp_ > tf_later.stamp_ || transform.stamp_ < tf_earlier.stamp_) {
    return false;
  }

  ros::Duration timeBetween = tf_later.stamp_ - tf_earlier.stamp_;
  ros::Duration timeAfterEarlier = transform.stamp_ - tf_earlier.stamp_;
  float tau = static_cast<float>(timeAfterEarlier.toNSec()) / timeBetween.toNSec();

  tf::Vector3 translation = tf_earlier.getOrigin() * (1.f - tau) + tf_later.getOrigin() * tau;
  tf::Quaternion rotation = tf_earlier.getRotation().slerp(tf_later.getRotation(), tau);

  transform.setOrigin(translation);
  transform.setRotation(rotation);
  return true;
}

bool TransformBuffer::insertTransform(const std::string& source_frame, const std::string& target_frame,
                                      tf::StampedTransform transform) {
  std::lock_guard<std::mutex> lck(mutex_);
  std::unordered_map<std::string, std::deque<tf::StampedTransform>>::iterator iterator =
      buffer_.find(getKey(source_frame, target_frame));
  if (iterator == buffer_.end()) {
    std::deque<tf::StampedTransform> empty_deque;
    buffer_[getKey(source_frame, target_frame)] = empty_deque;
    iterator = buffer_.find(getKey(source_frame, target_frame));
  }

  // check if the given transform is newer than the last buffered one
  if (iterator->second.size() == 0 || iterator->second.back().stamp_ < transform.stamp_) {
    iterator->second.push_back(transform);
    // remove transforms which are outside the buffer size
    while (transform.stamp_ - iterator->second.front().stamp_ > buffer_size_) {
      iterator->second.pop_front();
    }
    return true;
  }
  return false;
}

bool TransformBuffer::getTransform(const std::string& source_frame, const std::string& target_frame,
                                   const ros::Time& time, tf::StampedTransform& transform) const {
  std::lock_guard<std::mutex> lck(mutex_);
  std::unordered_map<std::string, std::deque<tf::StampedTransform>>::const_iterator iterator =
      buffer_.find(getKey(source_frame, target_frame));
  if (iterator == buffer_.end()) {
    print(log_level::error, "TF Buffer: could not retrieve requested transform from buffer, unregistered");
    return false;
  } else if (iterator->second.size() == 0) {
    print(log_level::warn, "TF Buffer: could not retrieve requested transform from buffer, buffer is empty");
    return false;
  } else {
    if (iterator->second.back().stamp_ < time) {
      print(log_level::debug, "TF Buffer: could not retrieve requested transform from buffer, tf has not yet arrived");
      return false;
    } else if (iterator->second.front().stamp_ > time) {
      print(log_level::warn,
            "TF Buffer: could not retrieve requested transform from buffer, tf has already been dropped from buffer");
      return false;
    } else {
      const tf::StampedTransform* previous = &iterator->second.back();
      for (std::deque<tf::StampedTransform>::const_reverse_iterator it = ++iterator->second.rbegin();
           it != iterator->second.rend(); ++it) {
        if (it->stamp_ <= time) {
          const tf::StampedTransform& tf_earlier = *it;
          const tf::StampedTransform& tf_later = *previous;
          transform.stamp_ = time;
          if (interpolateTransform(tf_earlier, tf_later, transform)) {
            return true;
          } else {
            print(log_level::warn, "TF Buffer: could not interpolate transform");
            return false;
          }
        }
        previous = &(*it);
      }
    }
  }
  return false;
}

void TransformBuffer::print(const log_level& level, const std::string& msg) const {
  if (ros::Time::now() - startup_time_ > ros::Duration(3)) {
    switch (level) {
      case error: {
        ROS_ERROR("%s", msg.c_str());
        break;
      }
      case warn: {
        ROS_WARN("%s", msg.c_str());
        break;
      }
      case info: {
        ROS_INFO("%s", msg.c_str());
        break;
      }
      case debug: {
        ROS_DEBUG("%s", msg.c_str());
        break;
      }
    }
  }
}
}
}
