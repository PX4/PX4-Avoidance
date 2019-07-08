#pragma once

#include <float.h>
#include <math.h>
#include <tf/transform_listener.h>
#include <deque>
#include <mutex>
#include <string>
#include <unordered_map>

namespace avoidance {

namespace tf_buffer {

enum log_level { error, warn, info, debug };

class TransformBuffer {
 public:
  TransformBuffer(float buffer_size_s = 10.0f);
  virtual ~TransformBuffer() = default;

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
  bool getTransform(const std::string& source_frame, const std::string& target_frame, const ros::Time& time,
                    tf::StampedTransform& transform) const;

 protected:
  std::unordered_map<std::string, std::deque<tf::StampedTransform>> buffer_;
  mutable std::mutex mutex_;
  ros::Duration buffer_size_;
  ros::Time startup_time_;

  /**
  * @brief      gets a key word from two frame names to identify a transform
  * @param[in]  source_frame, name of the source frame
  * @param[in]  target_frame, name of the target frame
  * @returns    key string
  **/
  std::string getKey(const std::string& source_frame, const std::string& target_frame) const;

  /**
  * @brief      interpolates between transforms
  * @param[in]  tf_earlier
  * @param[in]  tf_later
  * @param[in/out] transform, [in]  empty transform stamped with DESIRED TIMESTAMP
  *                           [out] correctly interpolated tf if interpolation is possible
  *                                 otherwise transform remains unchanged
  * @returns    bool, true if the transform was correctly interpolated
  **/
  bool interpolateTransform(const tf::StampedTransform& tf_earlier, const tf::StampedTransform& tf_later,
                            tf::StampedTransform& transform) const;

  /**
  * @brief      prints a message
  * @param[in]  level, valid options are: error, warn, info, debug
  * @param[in]  msg, string that should be printed
  **/
  void print(const log_level& level, const std::string& msg) const;
};
}
}
