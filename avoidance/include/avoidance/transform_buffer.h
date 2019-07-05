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

  /**
  * @brief      gets a key word from two frame names to identify a transform
  * @param[in]  source_frame, name of the source frame
  * @param[in]  target_frame, name of the target frame
  * @returns    key string
  **/
  std::string get_key(const std::string& source_frame, const std::string& target_frame) const;

  /**
  * @brief      interpolates between transforms
  * @param[in]  tf_earlier
  * @param[in]  tf_later
  * @param[out] transform, empty transform containing the desired timestep
  * @returns    bool, true if the transform can be interpolated
  **/
  bool interpolateTransform(const tf::StampedTransform& tf_earlier, const tf::StampedTransform& tf_later,
                            tf::StampedTransform& transform) const;

  /**
  * @brief      return whether a transform is registered with the buffer
  * @param[in]  source_frame
  * @param[in]  target_frame
  * @returns    bool, true if transform is registered
  **/
  bool isRegistered(const std::string& source_frame, const std::string& target_frame) const;

 public:
  std::vector<std::pair<std::string, std::string>> registered_transforms_;

  TransformBuffer(float buffer_size_s = 10.0f);
  ~TransformBuffer() = default;

  /**
  * @brief      initialized a new deque in the map to later store transforms
  * @param[in]  source_frame
  * @param[in]  target_frame
  **/
  void registerTransform(const std::string& source_frame, const std::string& target_frame);

  /**
  * @brief      inserts transform into buffer
  * @param[in]  source_frame
  * @param[in]  target_frame
  * @param[in]  transform
  * @returns    bool, true if transform was inserted (will not be inserted
  *             if it is the same or older than the last one that was buffered)
  **/
  bool insertTransform(const std::string& source_frame, const std::string& target_frame,
                       tf::StampedTransform transform);

  /**
  * @brief      retrieves transform from buffer
  * @param[in]  source_frame
  * @param[in]  target_frame
  * @param[in]  time, timestamp of the requested transform
  * @param[out] transform
  * @returns    bool, true if the transform could be retrieved from the buffer
  **/
  bool getTransform(const std::string& source_frame, const std::string& target_frame, ros::Time time,
                    tf::StampedTransform& transform) const;
};
}
