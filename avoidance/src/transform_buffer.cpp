#include "avoidance/transform_buffer.h"

namespace avoidance {

TransformBuffer::TransformBuffer(float buffer_size_s) : buffer_size_(ros::Duration(buffer_size_s)) {
  mutex_.reset(new std::mutex);
};

std::string TransformBuffer::get_key(const std::string& source_frame, const std::string& target_frame) const {
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

  tf::Vector3 translation = tf_earlier.getOrigin() * (1 - tau) + tf_later.getOrigin() * tau;
  tf::Quaternion rotation = tf_earlier.getRotation().slerp(tf_later.getRotation(), tau);

  transform.setOrigin(translation);
  transform.setRotation(rotation);
  return true;
}

bool TransformBuffer::isRegistered(const std::string& source_frame, const std::string& target_frame) const {
  std::unordered_map<std::string, std::deque<tf::StampedTransform>>::const_iterator iterator =
      buffer_.find(get_key(source_frame, target_frame));
  if (iterator == buffer_.end()) {
    return false;
  } else {
    return true;
  }
}

void TransformBuffer::registerTransform(const std::string& source_frame, const std::string& target_frame) {
  std::lock_guard<std::mutex> lck(*mutex_);
  if (!isRegistered(source_frame, target_frame)) {
    std::deque<tf::StampedTransform> empty_deque;
    std::pair<std::string, std::string> transform_frames;
    transform_frames.first = source_frame;
    transform_frames.second = target_frame;
    registered_transforms_.push_back(transform_frames);
    buffer_[get_key(source_frame, target_frame)] = empty_deque;
    ROS_WARN("transform buffer: Registered %s", get_key(source_frame, target_frame).c_str());
  }
}

bool TransformBuffer::insertTransform(const std::string& source_frame, const std::string& target_frame,
                                      tf::StampedTransform transform) {
  std::lock_guard<std::mutex> lck(*mutex_);
  std::unordered_map<std::string, std::deque<tf::StampedTransform>>::iterator iterator =
      buffer_.find(get_key(source_frame, target_frame));
  if (iterator == buffer_.end()) {
    ROS_ERROR("TF cannot be written to buffer, unregistered transform");
  } else {
    // check if the given transform is newer than the last buffered one
    bool new_tf = false;
    if (iterator->second.size() == 0) {
      new_tf = true;
    } else if (iterator->second.back().stamp_ < transform.stamp_) {
      new_tf = true;
    }

    if (new_tf) {
      iterator->second.push_back(transform);
      // remove transforms which are outside the buffer size
      while (transform.stamp_ - iterator->second.front().stamp_ > buffer_size_) {
        iterator->second.pop_front();
      }
      return true;
    }
  }
  return false;
}

bool TransformBuffer::getTransform(const std::string& source_frame, const std::string& target_frame, ros::Time time,
                                   tf::StampedTransform& transform) const {
  std::lock_guard<std::mutex> lck(*mutex_);
  std::unordered_map<std::string, std::deque<tf::StampedTransform>>::const_iterator iterator =
      buffer_.find(get_key(source_frame, target_frame));
  if (iterator == buffer_.end()) {
    ROS_ERROR("could not retrieve requested transform from buffer, unregistered");
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
            ROS_WARN("could not interpolate transform");
            return false;
          }
        }
        previous = &(*it);
      }
    }
  }
  return false;
}
}
