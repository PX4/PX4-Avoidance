#include <gtest/gtest.h>
#include <limits>
#include "avoidance/transform_buffer.h"

using namespace avoidance;

class TransformBufferTests : public ::testing::Test,
                             public TransformBuffer {
};

TEST(TransformBuffer, registerTransform) {
  // GIVEN: a transform Buffer and source/target frames
  TransformBuffer tf_buffer(10);
  std::string source_frame = "frame1";
  std::string target_frame = "frame2";

  // THEN: the buffer should not yet have this transform registered
  EXPECT_EQ(tf_buffer.registered_transforms_.size(), 0);

  // WHEN: we register that transform
  tf_buffer.registerTransform(source_frame, target_frame);

  // THEN: the buffer should have this transform registered
  EXPECT_EQ(tf_buffer.registered_transforms_.size(), 1);
  EXPECT_EQ(tf_buffer.registered_transforms_[0].first, source_frame);
  EXPECT_EQ(tf_buffer.registered_transforms_[0].second, target_frame);
}

TEST_F(TransformBufferTests, insertAndGetTransform) {
  // GIVEN: a transform Buffer and source/target frames
  std::string source_frame = "frame1";
  std::string target_frame = "frame2";

  ros::Time::init();
  ros::Time time1 = ros::Time::now();
  ros::Time time2 = time1 - ros::Duration(2);
  ros::Time time3 = time1 - ros::Duration(4);
  ros::Time time_between = time1 - ros::Duration(1.5);
  ros::Time time_before = time1 - ros::Duration(6);
  ros::Time time_after = time1 + ros::Duration(1);

  tf::StampedTransform transform1, transform2, transform3, retrieved_transform;
  transform1.stamp_ = time1;
  transform1.setIdentity();
  transform1.setOrigin({0, 0, 0});
  transform2.stamp_ = time2;
  transform2.setIdentity();
  transform2.setOrigin({0, 0, 0});
  transform3.stamp_ = time3;
  transform3.setIdentity();
  transform3.setOrigin({0, 0, 0});

  // WHEN: we register that transform
  registerTransform(source_frame, target_frame);
  insertTransform(source_frame, target_frame, transform3);
  insertTransform(source_frame, target_frame, transform2);
  insertTransform(source_frame, target_frame, transform1);

  // THEN: the buffer should have this transform registered
  EXPECT_TRUE(isRegistered(source_frame, target_frame));
  EXPECT_EQ(buffer_[get_key(source_frame, target_frame)].size(), 3);

  // time1 should get time1
  EXPECT_TRUE(getTransform(source_frame, target_frame, time1,
                                     retrieved_transform));
 EXPECT_EQ(retrieved_transform.stamp_, time1);

 // time2 should get time2
 EXPECT_TRUE(getTransform(source_frame, target_frame, time2,
                                    retrieved_transform));
 EXPECT_EQ(retrieved_transform.stamp_, time2);

 // time3 should get time3
 EXPECT_TRUE(getTransform(source_frame, target_frame, time3,
                                    retrieved_transform));
 EXPECT_EQ(retrieved_transform.stamp_, time3);

 // in between should give the timestamp of what we ask for
 EXPECT_TRUE(getTransform(source_frame, target_frame, time_between,
                                    retrieved_transform));
 EXPECT_EQ(retrieved_transform.stamp_, time_between);

 // outside of the buffer should not give a transform
 EXPECT_FALSE(getTransform(source_frame, target_frame, time_before,
                                     retrieved_transform));
 EXPECT_FALSE(getTransform(source_frame, target_frame, time_after,
                                     retrieved_transform));
}

TEST_F(TransformBufferTests, interpolateTransform) {
 // GIVEN: a transform Buffer and source/target frames
 std::string source_frame = "frame1";
 std::string target_frame = "frame2";

 ros::Time::init();
 ros::Time time1 = ros::Time::now();
 ros::Time time_half = time1 + ros::Duration(1);
 ros::Time time2 = time1 + ros::Duration(2);


 tf::StampedTransform transform1, transform2, retrieved_transform;
 transform1.stamp_ = time1;
 transform2.stamp_ = time2;
 retrieved_transform.stamp_ = time_half;
 tf::Vector3 translation1 = {0, 0, 0};
 tf::Vector3 translation2 = {0, 0, 2};
 tf::Vector3 translation_half = {0, 0, 1};
 transform1.setIdentity();
 transform2.setIdentity();

 transform1.setOrigin(translation1);
 transform2.setOrigin(translation2);

 ASSERT_TRUE(interpolateTransform(transform1, transform2, retrieved_transform));
 EXPECT_EQ(retrieved_transform.getOrigin(), translation_half);

}
