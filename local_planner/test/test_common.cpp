#include <gtest/gtest.h>
#include <limits>
#include "../src/nodes/common.h"

using namespace avoidance;

geometry_msgs::Point createPoint(const double& x, const double& y, const double& z){
  geometry_msgs::Point retval;
  retval.x = x;
  retval.y = y;
  retval.z = z;
  return retval;
}

TEST(Common, polar2DdistanceSameIsZero) {
  // GIVEN: two identical points
  const int e = 5, z = 9;

  // WHEN: we get the distance between the same points
  float dist = distance2DPolar(e, z, e, z);

  // THEN: the distance should be zero
  EXPECT_FLOAT_EQ(0.f, dist);
}

TEST(Common, polar2DdistanceOnKnownPoints) {
  // GIVEN: two points
  const int e1 = 5, z1 = 9;
  const int e2 = 50, z2 = 39;

  // WHEN: we get the distance between the same points
  float dist = distance2DPolar(e1, z1, e2, z2);

  // THEN: the distance should be...
  EXPECT_FLOAT_EQ(54.083271f, dist);
}


TEST(Common, azimuthAnglefromCartesian){
  //GIVEN:
  const geometry_msgs::Point point_right = createPoint(1.0d, 0.0d, 0.0d);
  const geometry_msgs::Point point_up = createPoint(0.0d, 1.0d, 0.0d);
  const geometry_msgs::Point point_left = createPoint(-1.0d, 0.0d, 0.0d);
  const geometry_msgs::Point point_down = createPoint(0.0d, -1.0d, 0.0d);
  const geometry_msgs::Point point_q1 = createPoint(1.0d, 2.0d, 0.0d);
  const geometry_msgs::Point point_q2 = createPoint(-3.0d, 4.0d, 0.0d);
  const geometry_msgs::Point point_q3 = createPoint(-5.0d, -6.0d, 0.0d);
  const geometry_msgs::Point point_q4 = createPoint(7.0d, -8.0d, 0.0d);
  const geometry_msgs::Point origin = createPoint(0.0d, 0.0d, 0.0d);

  //WHEN:
  int angle_right = azimuthAnglefromCartesian(point_right, origin);
  int angle_up = azimuthAnglefromCartesian(point_up, origin);
  int angle_left = azimuthAnglefromCartesian(point_left, origin);
  int angle_down = azimuthAnglefromCartesian(point_down, origin);
  int angle_undetermined = azimuthAnglefromCartesian(origin, origin);
  int angle_q1 = azimuthAnglefromCartesian(point_q1, origin);
  int angle_q2 = azimuthAnglefromCartesian(point_q2, origin);
  int angle_q3 = azimuthAnglefromCartesian(point_q3, origin);
  int angle_q4 = azimuthAnglefromCartesian(point_q4, origin);
  int angle_non_zero_origin = azimuthAnglefromCartesian(point_q1, point_q2);

  //THEN:
  EXPECT_EQ(90, angle_right);
  EXPECT_EQ(0, angle_up);
  EXPECT_EQ(180, angle_down);
  EXPECT_EQ(-90, angle_left);
  EXPECT_EQ(0, angle_undetermined);
  EXPECT_EQ(26, angle_q1);
  EXPECT_EQ(-37, angle_q2);
  EXPECT_EQ(-141, angle_q3);
  EXPECT_EQ(138, angle_q4);
  EXPECT_EQ(116, angle_non_zero_origin);
}

TEST(Common, elevationAnglefromCartesian){
  //GIVEN:
  const geometry_msgs::Point point_front = createPoint(0.0d, 1.0d, 0.0d);
  const geometry_msgs::Point point_up = createPoint(0.0d, 0.0d, 1.0d);
  const geometry_msgs::Point point_behind = createPoint(0.0d, -1.0d, 0.0d);
  const geometry_msgs::Point point_down = createPoint(0.0d, 0.0d, -1.0d);
  const geometry_msgs::Point point_q1 = createPoint(0.0d, 1.0d, 2.0d);
  const geometry_msgs::Point point_q2 = createPoint(0.0d, -3.0d, 4.0d);
  const geometry_msgs::Point point_q3 = createPoint(0.0d, -5.0d, -6.0d);
  const geometry_msgs::Point point_q4 = createPoint(0.0d, 7.0d, -8.0d);
  const geometry_msgs::Point origin = createPoint(0.0d, 0.0d, 0.0d);

  //WHEN:
  const int angle_front = elevationAnglefromCartesian(point_front, origin);
  const int angle_up = elevationAnglefromCartesian(point_up, origin);
  const int angle_behind = elevationAnglefromCartesian(point_behind, origin);
  const int angle_down = elevationAnglefromCartesian(point_down, origin);
  const int angle_undetermined = elevationAnglefromCartesian(origin, origin);
  const int angle_q1 = elevationAnglefromCartesian(point_q1, origin);
  const int angle_q2 = elevationAnglefromCartesian(point_q2, origin);
  const int angle_q3 = elevationAnglefromCartesian(point_q3, origin);
  const int angle_q4 = elevationAnglefromCartesian(point_q4, origin);
  const int angle_non_zero_origin = elevationAnglefromCartesian(point_q1, point_q2);

  //THEN:
  EXPECT_EQ(0, angle_front);
  EXPECT_EQ(0, angle_up);
  EXPECT_EQ(0, angle_behind);
  EXPECT_EQ(0, angle_down);
  EXPECT_EQ(0, angle_undetermined);
  EXPECT_EQ(63, angle_q1);
  EXPECT_EQ(53, angle_q2);
  EXPECT_EQ(-51, angle_q3);
  EXPECT_EQ(-49, angle_q4);
  EXPECT_EQ(-27, angle_non_zero_origin);
}

TEST(Common, elevationAngletoIndex){
  //GIVEN:
  const int elevation_1 = 0;
  const int elevation_2 = 34;
  const int resolution_1 = 3;
  const int resolution_2 = 12;
  const int elevation_invalid_1 = 94;
  const int elevation_invalid_2 = -999;
  const int resolution_invalid_1 = 0;
  const int resolution_invalid_2 = -1;

  //WHEN:
  const int index_1 = elevationAngletoIndex(elevation_1, resolution_1);
  const int index_2 = elevationAngletoIndex(elevation_2, resolution_1);
  const int index_3 = elevationAngletoIndex(elevation_1, resolution_2);
  const int index_4 = elevationAngletoIndex(elevation_2, resolution_2);
  const int index_invalid_1 = elevationAngletoIndex(elevation_invalid_1, resolution_1);
  const int index_invalid_2 = elevationAngletoIndex(elevation_invalid_2, resolution_1);
  const int index_invalid_3 = elevationAngletoIndex(elevation_1, resolution_invalid_1);
  const int index_invalid_4 = elevationAngletoIndex(elevation_1, resolution_invalid_2);

  //THEN:
  EXPECT_EQ(30, index_1);
  EXPECT_EQ(41, index_2);
  EXPECT_EQ(7, index_3);
  EXPECT_EQ(10, index_4);
  EXPECT_EQ(0, index_invalid_1);
  EXPECT_EQ(0, index_invalid_2);
  EXPECT_EQ(0, index_invalid_3);
  EXPECT_EQ(0, index_invalid_4);

}
