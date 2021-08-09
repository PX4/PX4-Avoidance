#include "avoidance/transform_buffer.h"

namespace avoidance {

namespace tf_buffer {

TransformBuffer::TransformBuffer(float buffer_size_s) : buffer_size_(rclcpp::Duration(buffer_size_s)) {
  startup_time_ = rclcpp::Clock().now();
}

std::string TransformBuffer::getKey(const std::string& source_frame, const std::string& target_frame) const {
  return source_frame + "_to_" + target_frame;
}

bool TransformBuffer::interpolateTransform(const geometry_msgs::msg::TransformStamped& tf_earlier, const geometry_msgs::msg::TransformStamped& tf_later,
                                           geometry_msgs::msg::TransformStamped& transform) const {
  // check if the requested timestamp lies between the two given transforms
  if (rclcpp::Time(transform.header.stamp) > rclcpp::Time(tf_later.header.stamp) || rclcpp::Time(transform.header.stamp) < rclcpp::Time(tf_earlier.header.stamp)) {
    return false;
  }

  const rclcpp::Duration timeBetween = rclcpp::Time(tf_later.header.stamp) - rclcpp::Time(tf_earlier.header.stamp);
  const rclcpp::Duration timeAfterEarlier = rclcpp::Time(transform.header.stamp) - rclcpp::Time(tf_earlier.header.stamp);
  const float tau = static_cast<float>(timeAfterEarlier.nanoseconds()) / timeBetween.nanoseconds();

  tf2::Vector3 tf_earlier_translation;
  tf2::Quaternion tf_earlier_rotation;
  tf2::Quaternion tf_later_rotation;
  tf_earlier_translation.setValue(tf_earlier.transform.translation.x, tf_earlier.transform.translation.y, tf_earlier.transform.translation.z);
  // tf2::fromMsg(tf_earlier.transform.translation, tf_earlier_translation);
  tf2::fromMsg(tf_earlier.transform.rotation, tf_earlier_rotation);
  tf2::fromMsg(tf_later.transform.rotation, tf_later_rotation);

  const tf2::Vector3 translation = tf_earlier_translation * (1.f - tau) + tf_earlier_translation * tau;
  const tf2::Quaternion rotation = tf_earlier_rotation.slerp(tf_later_rotation, tau);

  transform.transform.translation = avoidance::toVector3Msg(translation);
  transform.transform.rotation = tf2::toMsg(rotation);
  return true;
}

bool TransformBuffer::insertTransform(const std::string& source_frame, const std::string& target_frame,
                                      geometry_msgs::msg::TransformStamped transform) {
  std::lock_guard<std::mutex> lck(mutex_);
  std::unordered_map<std::string, std::deque<geometry_msgs::msg::TransformStamped>>::iterator iterator =
      buffer_.find(getKey(source_frame, target_frame));
  if (iterator == buffer_.end()) {
    std::deque<geometry_msgs::msg::TransformStamped> empty_deque;
    buffer_[getKey(source_frame, target_frame)] = empty_deque;
    iterator = buffer_.find(getKey(source_frame, target_frame));
  }

  // check if the given transform is newer than the last buffered one
  if (iterator->second.size() == 0 || rclcpp::Time(iterator->second.back().header.stamp) < rclcpp::Time(transform.header.stamp)) {
    iterator->second.push_back(transform);
    // remove transforms which are outside the buffer size
    while (rclcpp::Time(transform.header.stamp) - rclcpp::Time(iterator->second.front().header.stamp) > buffer_size_) {
      iterator->second.pop_front();
    }
    return true;
  }
  return false;
}

bool TransformBuffer::getTransform(const std::string& source_frame, const std::string& target_frame,
                                   const rclcpp::Time& time, geometry_msgs::msg::TransformStamped& transform) const {
  std::lock_guard<std::mutex> lck(mutex_);
  std::unordered_map<std::string, std::deque<geometry_msgs::msg::TransformStamped>>::const_iterator iterator =
      buffer_.find(getKey(source_frame, target_frame));
  if (iterator == buffer_.end()) {
    print(log_level::error, "TF Buffer: could not retrieve requested transform from buffer, unregistered");
    return false;
  } else if (iterator->second.size() == 0) {
    print(log_level::warn, "TF Buffer: could not retrieve requested transform from buffer, buffer is empty");
    return false;
  } else {
    if (rclcpp::Time(iterator->second.back().header.stamp) < time) {
      print(log_level::debug, "TF Buffer: could not retrieve requested transform from buffer, tf has not yet arrived");
      return false;
    } else if (rclcpp::Time(iterator->second.front().header.stamp) > time) {
      print(log_level::warn,
            "TF Buffer: could not retrieve requested transform from buffer, tf has already been dropped from buffer");
      return false;
    } else {
      const geometry_msgs::msg::TransformStamped* previous = &iterator->second.back();
      for (std::deque<geometry_msgs::msg::TransformStamped>::const_reverse_iterator it = ++iterator->second.rbegin();
           it != iterator->second.rend(); ++it) {
        if (rclcpp::Time(it->header.stamp) <= time) {
          const geometry_msgs::msg::TransformStamped& tf_earlier = *it;
          const geometry_msgs::msg::TransformStamped& tf_later = *previous;
          transform.header.stamp = time;
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
  if (rclcpp::Clock().now() - startup_time_ > rclcpp::Duration(3)) {
    switch (level) {
      case error: {
        RCLCPP_ERROR(tf_logger_, "%s", msg.c_str());
        break;
      }
      case warn: {
        RCLCPP_WARN(tf_logger_, "%s", msg.c_str());
        break;
      }
      case info: {
        RCLCPP_INFO(tf_logger_, "%s", msg.c_str());
        break;
      }
      case debug: {
        RCLCPP_DEBUG(tf_logger_, "%s", msg.c_str());
        break;
      }
    }
  }
}
}
}
