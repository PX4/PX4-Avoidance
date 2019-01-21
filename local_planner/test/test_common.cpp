#include <gtest/gtest.h>
#include <limits>
#include "../src/nodes/common.h"
#include "../src/nodes/histogram.h"

using namespace avoidance;

TEST(PlannerFunctions, HistogramResolution) {
  // Test that the hardcoded histogram resolution ALPHA_RES is valid
  ASSERT_GT(ALPHA_RES, 0);
  ASSERT_EQ(180 % (2 * ALPHA_RES), 0);
}

TEST(Common, polar2DdistanceSameIsZero) {
  // GIVEN: two identical points
  PolarPoint p(5.0f, 9.0f, 0.0f);

  // WHEN: we get the distance between the same points
  float dist = distance2DPolar(p, p);

  // THEN: the distance should be zero
  EXPECT_FLOAT_EQ(0.f, dist);
}

TEST(Common, polar2DdistanceOnKnownPoints) {
  // GIVEN: two points
  PolarPoint p1(5.0f, 9.0f, 0.0f);
  PolarPoint p2(50.0f, 39.0f, 0.0f);

  // WHEN: we get the distance between the same points
  float dist = distance2DPolar(p1, p2);

  // THEN: the distance should be...
  EXPECT_FLOAT_EQ(54.083271f, dist);
}

TEST(Common, indexAngleDifferenceCheck) {
  // GIVEN: two angles
  const float a1 = 0.f, b1 = 0.f;
  const float a2 = 180.f, b2 = -180.f;
  const float a3 = 0.f, b3 = 360.f;
  const float a4 = 8.45860f, b4 = 9.45859f;

  // WHEN: we get the minimal different angle between the two angles
  double angle_diff1 = indexAngleDifference(a1, b1);
  double angle_diff2 = indexAngleDifference(a2, b2);
  double angle_diff3 = indexAngleDifference(a3, b3);
  double angle_diff4 = indexAngleDifference(a4, b4);

  // THEN: the angle should be...
  EXPECT_FLOAT_EQ(0., angle_diff1);
  EXPECT_FLOAT_EQ(0., angle_diff2);
  EXPECT_FLOAT_EQ(0., angle_diff3);
  ASSERT_NEAR(0.99999, angle_diff4, .00001);
}

TEST(Common, azimuthAnglefromCartesian) {
  // GIVEN: two points
  const Eigen::Vector3f point_right(1.0d, 0.0d, 0.0d);
  const Eigen::Vector3f point_up(0.0d, 1.0d, 0.0d);
  const Eigen::Vector3f point_left(-1.0d, 0.0d, 0.0d);
  const Eigen::Vector3f point_down(0.0d, -1.0d, 0.0d);
  const Eigen::Vector3f point_q1(1.0d, 2.0d, 0.0d);
  const Eigen::Vector3f point_q2(-3.0d, 4.0d, 0.0d);
  const Eigen::Vector3f point_q3(-5.0d, -6.0d, 0.0d);
  const Eigen::Vector3f point_q4(7.0d, -8.0d, 0.0d);
  const Eigen::Vector3f origin(0.0d, 0.0d, 0.0d);

  // WHEN: calculating the azimuth angle between the two points
  float angle_right = cartesianToPolar(point_right, origin).z;
  float angle_up = cartesianToPolar(point_up, origin).z;
  float angle_left = cartesianToPolar(point_left, origin).z;
  float angle_down = cartesianToPolar(point_down, origin).z;
  float angle_undetermined = cartesianToPolar(origin, origin).z;
  float angle_q1 = cartesianToPolar(point_q1, origin).z;
  float angle_q2 = cartesianToPolar(point_q2, origin).z;
  float angle_q3 = cartesianToPolar(point_q3, origin).z;
  float angle_q4 = cartesianToPolar(point_q4, origin).z;
  float angle_non_zero_origin = cartesianToPolar(point_q1, point_q2).z;

  // THEN:  angle should be ..
  EXPECT_FLOAT_EQ(90, angle_right);
  EXPECT_FLOAT_EQ(0, angle_up);
  EXPECT_FLOAT_EQ(180, angle_down);
  EXPECT_FLOAT_EQ(-90, angle_left);
  EXPECT_FLOAT_EQ(0, angle_undetermined);
  EXPECT_FLOAT_EQ(26.565051, angle_q1);
  EXPECT_FLOAT_EQ(-36.869897, angle_q2);
  EXPECT_FLOAT_EQ(-140.194428, angle_q3);
  EXPECT_FLOAT_EQ(138.814074, angle_q4);
  EXPECT_FLOAT_EQ(116.565051, angle_non_zero_origin);
}

TEST(Common, elevationAnglefromCartesian) {
  // GIVEN: two points
  const Eigen::Vector3f point_front(0.0d, 1.0d, 0.0d);
  const Eigen::Vector3f point_up(0.0d, 0.0d, 1.0d);
  const Eigen::Vector3f point_behind(0.0d, -1.0d, 0.0d);
  const Eigen::Vector3f point_down(0.0d, 0.0d, -1.0d);
  const Eigen::Vector3f point_q1(0.0d, 1.0d, 1.732050808d);
  const Eigen::Vector3f point_30(0.0d, 1.0d, 0.577350269d);
  const Eigen::Vector3f point_q2(0.0d, -3.0d, 4.0d);
  const Eigen::Vector3f point_q3(0.0d, -5.0d, -6.0d);
  const Eigen::Vector3f point_q4(0.0d, 7.0d, -8.0d);
  const Eigen::Vector3f origin(0.0d, 0.0d, 0.0d);

  // WHEN: we get the elevation angle between the two points
  const float angle_front = cartesianToPolar(point_front, origin).e;
  const float angle_up = cartesianToPolar(point_up, origin).e;
  const float angle_behind = cartesianToPolar(point_behind, origin).e;
  const float angle_down = cartesianToPolar(point_down, origin).e;
  const float angle_undetermined = cartesianToPolar(origin, origin).e;
  const float angle_q1 = cartesianToPolar(point_q1, origin).e;
  const float angle_30 = cartesianToPolar(point_30, origin).e;
  const float angle_q2 = cartesianToPolar(point_q2, origin).e;
  const float angle_q3 = cartesianToPolar(point_q3, origin).e;
  const float angle_q4 = cartesianToPolar(point_q4, origin).e;
  const float angle_non_zero_origin = cartesianToPolar(point_q4, point_q2).e;

  // THEN: angle should be ..
  EXPECT_FLOAT_EQ(0.f, angle_front);
  EXPECT_FLOAT_EQ(90.f, angle_up);
  EXPECT_FLOAT_EQ(0.f, angle_behind);
  EXPECT_FLOAT_EQ(-90.f, angle_down);
  EXPECT_FLOAT_EQ(0.f, angle_undetermined);
  EXPECT_FLOAT_EQ(60.f, angle_q1);
  EXPECT_FLOAT_EQ(30.f, angle_30);
  EXPECT_FLOAT_EQ(53.130110, angle_q2);
  EXPECT_FLOAT_EQ(-50.194429, angle_q3);
  EXPECT_FLOAT_EQ(-48.8140748, angle_q4);
  EXPECT_FLOAT_EQ(-50.194428, angle_non_zero_origin);
}

TEST(Common, polarToHistogramIndex) {
  // GIVEN: the polar point and the histogram resolution
  PolarPoint p_pol_1(0.f, 0.f, 0.f);
  PolarPoint p_pol_2(34.0f, 34.0f, 0.0f);
  PolarPoint p_pol_3(90.0f, 180.0f, 0.0f);
  PolarPoint p_pol_4(-90.0f, -180.0f, 0.0f);
  // wrapped around, influences the azimuth by 180 deg
  PolarPoint p_pol_5(454.f, -160.f, 0.0f);
  // wrapped around, no influence on azimuth
  PolarPoint p_pol_6(400.f, -270.f, 0.0f);

  const float resolution_1 = 3.f;
  const float resolution_2 = 12.f;

  // WHEN: we convert the polar point to a histogram index
  const Eigen::Vector2i index_1 = polarToHistogramIndex(p_pol_1, resolution_1);
  const Eigen::Vector2i index_2 = polarToHistogramIndex(p_pol_2, resolution_1);
  const Eigen::Vector2i index_3 = polarToHistogramIndex(p_pol_1, resolution_2);
  const Eigen::Vector2i index_4 = polarToHistogramIndex(p_pol_2, resolution_2);
  const Eigen::Vector2i index_5 = polarToHistogramIndex(p_pol_3, resolution_2);
  const Eigen::Vector2i index_6 = polarToHistogramIndex(p_pol_4, resolution_2);
  const Eigen::Vector2i index_7 =
      polarToHistogramIndex(p_pol_5, resolution_1);  // wrapped
  const Eigen::Vector2i index_8 =
      polarToHistogramIndex(p_pol_6, resolution_2);  // wrapped

  // THEN: the  histogram index should be ..
  // elevation angle
  EXPECT_EQ(30, index_1.y());
  EXPECT_EQ(41, index_2.y());
  EXPECT_EQ(7, index_3.y());
  EXPECT_EQ(10, index_4.y());
  EXPECT_EQ(14, index_5.y());
  EXPECT_EQ(0, index_6.y());
  EXPECT_EQ(58, index_7.y());
  EXPECT_EQ(10, index_8.y());
  // azimuth angle
  EXPECT_EQ(60, index_1.x());
  EXPECT_EQ(71, index_2.x());
  EXPECT_EQ(15, index_3.x());
  EXPECT_EQ(17, index_4.x());
  EXPECT_EQ(0, index_5.x());
  EXPECT_EQ(0, index_6.x());
  EXPECT_EQ(66, index_7.x());
  EXPECT_EQ(22, index_8.x());
}

TEST(Common, polarToCartesian) {
  // GIVEN: the elevation angle, azimuth angle, a radius and the position
  std::vector<float> e = {-90.f, -90.f, 90.f, 0.f, 45.f};    //[-90, 90]
  std::vector<float> z = {-180.f, -90.f, 179.f, 0.f, 45.f};  //[-180, 180]

  // Check that the input is valid
  int n = 0;
  if (e.size() == z.size()) {
    n = e.size();
  }
  ASSERT_GT(n, 0);

  std::vector<float> radius = {0.f, 2.f};

  Eigen::Vector3f pos(0.f, 0.f, 0.f);

  std::vector<Eigen::Vector3f> pos_out;

  // WHEN: converting the point in polar CS to cartesian CS

  for (int i = 0; i < n; i++) {
    PolarPoint p_pol(e[i], z[3], radius[0]);
    pos_out.push_back(polarToCartesian(p_pol, toPoint(pos)));
  }

  for (int i = 0; i < n; i++) {
    PolarPoint p_pol(e[i], z[i], radius[1]);
    pos_out.push_back(polarToCartesian(p_pol, toPoint(pos)));
  }

  // THEN: the cartesian coordinates are

  for (int i = 0; i < n; i++) {
    EXPECT_FLOAT_EQ(0.f, pos_out[i].x());
    EXPECT_FLOAT_EQ(0.f, pos_out[i].y());
    EXPECT_FLOAT_EQ(0.f, pos_out[i].z());
  }

  EXPECT_NEAR(-0.f, pos_out[5].x(), 0.00001);
  EXPECT_NEAR(-0.f, pos_out[5].y(), 0.00001);
  EXPECT_FLOAT_EQ(-2.f, pos_out[5].z());

  EXPECT_NEAR(-0.f, pos_out[6].x(), 0.00001);
  EXPECT_NEAR(0.f, pos_out[6].y(), 0.00001);
  EXPECT_FLOAT_EQ(-2.f, pos_out[6].z());

  EXPECT_NEAR(0.f, pos_out[7].x(), 0.00001);
  EXPECT_NEAR(-0.f, pos_out[7].y(), 0.00001);
  EXPECT_FLOAT_EQ(2.f, pos_out[7].z());

  EXPECT_NEAR(0.f, pos_out[8].x(), 0.00001);
  EXPECT_NEAR(2.f, pos_out[8].y(), 0.00001);
  EXPECT_NEAR(0.f, pos_out[8].z(), 0.00001);

  EXPECT_NEAR(1.f, pos_out[9].x(), 0.00001);
  EXPECT_NEAR(1.f, pos_out[9].y(), 0.00001);
  EXPECT_NEAR(1.414213562, pos_out[9].z(), 0.00001);
}

TEST(Common, PolarToCatesianToPolar) {
  // GIVEN: a current position and a radius
  double radius = 2.168;
  Eigen::Vector3f pos(4.21, 2.34, 0.19);

  // WHEN: going through all valid polar coordinates and transform it to
  // cartesian and back again
  for (float e = -90.f; e <= 90.f; e = e + 3.f) {
    for (float z = -180.f; z <= 180.f; z = z + 6.f) {
      PolarPoint p_pol(e, z, radius);
      Eigen::Vector3f p_cartesian = polarToCartesian(p_pol, toPoint(pos));

      PolarPoint p_pol_new = cartesianToPolar(p_cartesian, pos);

      // THEN: the resulting polar positions are expected to be the same as
      // before the conversion
      ASSERT_GE(p_pol_new.z, -180);
      ASSERT_LE(p_pol_new.z, 180);
      ASSERT_GE(p_pol_new.e, -90);
      ASSERT_LE(p_pol_new.e, 90);

      if (std::abs(std::abs(p_pol_new.e) - 90.f) > 1e-5) {
        if (std::abs(std::abs(p_pol_new.z) - 180.f) < 1e-5) {
          EXPECT_NEAR(std::abs(z), std::abs(p_pol_new.z), 0.001);
        } else {
          EXPECT_NEAR(z, p_pol_new.z, 0.001);
        }
      }
      EXPECT_NEAR(e, p_pol_new.e, 0.001);
    }
  }
}

TEST(Common, cartesianTopolarToCartesian) {
  // GIVEN: a current position
  Eigen::Vector3f pos(0.81, 5.17, 3.84);

  // WHEN: going through some valid cartesian coordinates and transform it to
  // polar and back to cartesian
  for (float x = -5.f; x <= 5.f; x = x + 0.5f) {
    for (float y = -5.f; y <= 5.f; y = y + 0.6f) {
      for (float z = -5.f; z <= 5.f; z = z + 0.4f) {
        Eigen::Vector3f origin(x, y, z);
        PolarPoint p_pol = cartesianToPolar(origin, pos);
        // p_pol.r = (origin - pos).norm();
        Eigen::Vector3f p_cartesian = polarToCartesian(p_pol, toPoint(pos));

        // THEN: the resulting cartesian positions are expected to be the same
        // as
        // before the conversion
        EXPECT_LT((origin - p_cartesian).norm(), 0.001);
      }
    }
  }
}

TEST(Common, nextYawAngle) {
  // GIVEN: two points
  geometry_msgs::PoseStamped location;
  location.pose.position.x = 0;
  location.pose.position.y = 0;
  location.pose.position.z = 0;
  Eigen::Vector3f next_pos1(1.0d, 1.0d, 0.0d);
  Eigen::Vector3f next_pos2(0.0d, 0.0d, 0.0d);

  // WHEN: we get the yaw between the two points
  double yaw1 = nextYaw(location, toPoint(next_pos1));
  double yaw2 = nextYaw(location, toPoint(next_pos2));

  // THEN: the angle in rad should be...
  EXPECT_NEAR(0.785398163, yaw1, 0.00001);
  EXPECT_NEAR(0.f, yaw2, 0.00001);
}

TEST(Common, speedCalc) {
  // GIVEN: maximum and minimum velocity, slope, old velocity and time elapsed
  double max_vel1 = 0.d;
  double max_vel2 = 10.d;
  double min_vel1 = 0.d;
  double min_vel2 = 2.d;
  double slope = 1.d;  // always one
  double v_old1 = 0.d;
  double v_old2 = 10.d;
  double elapsed = 2.d;

  // WHEN: we get distance between the same points
  double speed1 = velocityLinear(max_vel2, slope, v_old1, elapsed);
  double speed2 = velocityLinear(max_vel1, slope, v_old2, elapsed);
  double speed3 = velocityLinear(max_vel2, slope, v_old2, elapsed);

  // THEN: the distance should be...
  EXPECT_FLOAT_EQ(2.f, speed1);  // normal case
  EXPECT_FLOAT_EQ(0.f, speed2);  // max velocity is 0 and curr vel not
  EXPECT_FLOAT_EQ(
      10.f, speed3);  // calculated speed is higher than max vel, set to max vel
}

TEST(Common, wrapAngle) {
  // GIVEN: an angle in rad
  double angle1 = 0.d;
  double angle2 = 30.d * M_PI / 180.d;
  double angle3 = 270.d * M_PI / 180.d;
  double angle4 = -90.d * M_PI / 180.d;
  double angle5 = -225.d * M_PI / 180.d;

  // WHEN: it is wrapped to the space (-PI; PI] space
  wrapAngleToPlusMinusPI(angle1);
  wrapAngleToPlusMinusPI(angle2);
  wrapAngleToPlusMinusPI(angle3);
  wrapAngleToPlusMinusPI(angle4);
  wrapAngleToPlusMinusPI(angle5);
  // THEN: the output angles shoudl be ..
  EXPECT_FLOAT_EQ(0.f, angle1);
  EXPECT_FLOAT_EQ(0.523599f, angle2);
  EXPECT_FLOAT_EQ(-1.570796f, angle3);
  EXPECT_FLOAT_EQ(-1.570796f, angle4);
  EXPECT_FLOAT_EQ(2.356194f, angle5);
}

TEST(Common, getAngularVel) {
  // GIVEN: maximum and minimum velocity, slope, old velocity and time elapsed
  double desired_yaw1 = 0.d;
  double desired_yaw2 = 540.d * M_PI / 180.d;
  double curr_yaw1 = 0.d * M_PI / 180.d;
  double curr_yaw2 = -45.d * M_PI / 180.d;

  // WHEN: we get distance between the same points
  double angular_vel1 = getAngularVelocity(desired_yaw1, curr_yaw1);
  double angular_vel2 = getAngularVelocity(desired_yaw2, curr_yaw1);
  double angular_vel3 = getAngularVelocity(desired_yaw1, curr_yaw2);
  double angular_vel4 = getAngularVelocity(desired_yaw2, curr_yaw2);

  // THEN: the distance should be...

  EXPECT_FLOAT_EQ(0.f, angular_vel1);
  EXPECT_FLOAT_EQ(1.570796f, angular_vel2);
  EXPECT_FLOAT_EQ(0.392699f, angular_vel3);
  EXPECT_FLOAT_EQ(-1.178097f, angular_vel4);
}

TEST(Common, IndexPolarIndex) {
  // GIVEN:  some valid histogram indices and resolution
  int res = 6;

  for (int e_ind = 0; e_ind == 30; e_ind + 5) {
    for (int z_ind = 0; z_ind == 60; z_ind + 5) {
      // WHEN: transform it to polar and back to the indices
      PolarPoint p_pol = histogramIndexToPolar(e_ind, z_ind, res, 0.0);
      Eigen::Vector2i p_ind = polarToHistogramIndex(p_pol, res);

      // THEN: new histogram index should be the same as before the conversion
      EXPECT_EQ(e_ind, p_ind.y());
      EXPECT_EQ(z_ind, p_ind.x());
    }
  }
}

TEST(Common, wrapPolar) {
  // GIVEN: some polar points with elevation and azimuth angles which need to be
  // wrapped
  float rad = 1.0f;
  PolarPoint p_pol_1(110.f, 0.f, rad);   // wrap  elevation with jump in azimuth
  PolarPoint p_pol_2(0.f, 200.f, rad);   // wrap azimuth
  PolarPoint p_pol_3(-180.f, 0.f, rad);  // wrap elevation with jump in azimuth
  PolarPoint p_pol_4(0.f, -1230.f, rad);  // wrap azimuth multiple times
  // wrap elevation, no change in azimuth
  PolarPoint p_pol_5(-330.f, 140.f, rad);
  // wrap azimuth, no change in elevation
  PolarPoint p_pol_6(40.f, 600.f, rad);

  // WHEN: the angles are wrapped
  wrapPolar(p_pol_1);
  wrapPolar(p_pol_2);
  wrapPolar(p_pol_3);
  wrapPolar(p_pol_4);
  wrapPolar(p_pol_5);
  wrapPolar(p_pol_6);

  // THEN: the polar points should have the angles
  EXPECT_FLOAT_EQ(70.f, p_pol_1.e);
  EXPECT_FLOAT_EQ(-180.f, p_pol_1.z);

  EXPECT_FLOAT_EQ(0.f, p_pol_2.e);
  EXPECT_FLOAT_EQ(-160.f, p_pol_2.z);

  EXPECT_FLOAT_EQ(0.f, p_pol_3.e);
  EXPECT_FLOAT_EQ(-180.f, p_pol_3.z);

  EXPECT_FLOAT_EQ(0.f, p_pol_4.e);
  EXPECT_FLOAT_EQ(-150.f, p_pol_4.z);

  EXPECT_FLOAT_EQ(30.f, p_pol_5.e);
  EXPECT_FLOAT_EQ(140.f, p_pol_5.z);

  EXPECT_FLOAT_EQ(40.f, p_pol_6.e);
  EXPECT_FLOAT_EQ(-120.f, p_pol_6.z);
}