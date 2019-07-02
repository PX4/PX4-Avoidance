#include <gtest/gtest.h>
#include <limits>
#include "avoidance/transform_buffer.h"

using namespace avoidance;

TEST(TransformBuffer, registerTransform) {
  // GIVEN: a transform Buffer and source/target frames
  TransformBuffer tf_buffer(10);
  std::string source_frame = "frame1";
  std::string target_frame = "frame2";

  // THEN: the buffer should not yet have this transform registered
  EXPECT_FALSE(tf_buffer.isRegistered(source_frame, target_frame));
  EXPECT_EQ(tf_buffer.registered_transforms_.size(), 0);

  // WHEN: we register that transform
  tf_buffer.registerTransform(source_frame, target_frame);

  // THEN: the buffer should have this transform registered
  EXPECT_TRUE(tf_buffer.isRegistered(source_frame, target_frame));
  EXPECT_EQ(tf_buffer.registered_transforms_.size(), 1);
  EXPECT_EQ(tf_buffer.registered_transforms_[0].first, source_frame);
  EXPECT_EQ(tf_buffer.registered_transforms_[0].second, target_frame);
}

TEST(TransformBuffer, insertAndGetTransform) {
  // GIVEN: a transform Buffer and source/target frames
  TransformBuffer tf_buffer(10);
  std::string source_frame = "frame1";
  std::string target_frame = "frame2";

  ros::Time::init();
  ros::Time time1 = ros::Time::now();
  ros::Time time2 = time1 - ros::Duration(2);
  ros::Time time3 = time1 - ros::Duration(4);
  ros::Time time_between = time1 - ros::Duration(1);
  ros::Time time_before = time1 - ros::Duration(6);
  ros::Time time_after = time1 + ros::Duration(1);

  tf::StampedTransform transform1, transform2, transform3, retrieved_transform;
  transform1.stamp_ = time1;
  transform2.stamp_ = time2;
  transform3.stamp_ = time3;

  // WHEN: we register that transform
  tf_buffer.registerTransform(source_frame, target_frame);
  tf_buffer.insertTransform(source_frame, target_frame, transform3);
  tf_buffer.insertTransform(source_frame, target_frame, transform2);
  tf_buffer.insertTransform(source_frame, target_frame, transform1);

  // THEN: the buffer should have this transform registered
  EXPECT_TRUE(tf_buffer.isRegistered(source_frame, target_frame));
  EXPECT_TRUE(tf_buffer.getTransform(source_frame, target_frame, time1,
                                     retrieved_transform));
  // EXPECT_LT(retrieved_transform.stamp_, time1 + ros::Duration(0.001));
  // EXPECT_GT(retrieved_transform.stamp_, time1 - ros::Duration(0.001));
  EXPECT_TRUE(tf_buffer.getTransform(source_frame, target_frame, time2,
                                     retrieved_transform));
  EXPECT_TRUE(tf_buffer.getTransform(source_frame, target_frame, time3,
                                     retrieved_transform));
  EXPECT_TRUE(tf_buffer.getTransform(source_frame, target_frame, time_between,
                                     retrieved_transform));
  EXPECT_FALSE(tf_buffer.getTransform(source_frame, target_frame, time_before,
                                      retrieved_transform));
  EXPECT_FALSE(tf_buffer.getTransform(source_frame, target_frame, time_after,
                                      retrieved_transform));
}
