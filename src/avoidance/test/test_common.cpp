#include <gtest/gtest.h>
#include <limits>
#include "avoidance/common.h"
#include "avoidance/histogram.h"

using namespace avoidance;

TEST(Common, pointInsideFOV) {
  // GIVEN: three points and a regular FOV
  FOV fov(34.0f, 12.0f, 90.0f, 60.0f);
  PolarPoint p_outside_azimuth(-1.0f, 126.0f, 2.0f);
  PolarPoint p_outside_elevation(-60.0f, 30.0f, 2.0f);
  PolarPoint p_inside_elevation_sign(41.0f, 0.0f, 2.0f);
  PolarPoint p_inside_fov(5.0f, 25.0f, 2.0f);

  // WHEN: we check whether they are inside the FOV
  bool outside_azimuth = pointInsideFOV(fov, p_outside_azimuth);
  bool outside_elevation = pointInsideFOV(fov, p_outside_elevation);
  bool inside_elevation_sign = pointInsideFOV(fov, p_inside_elevation_sign);
  bool inside = pointInsideFOV(fov, p_inside_fov);

  // THEN: they should lie in the expected region
  EXPECT_FALSE(outside_azimuth);
  EXPECT_FALSE(outside_elevation);
  EXPECT_TRUE(inside_elevation_sign);
  EXPECT_TRUE(inside);
}

TEST(Common, pointInsideFOVvector) {
  // GIVEN: three points and a regular FOV
  FOV fov1(34.0f, 12.0f, 45.0f, 60.0f);
  FOV fov2(80.0f, 12.0f, 45.0f, 60.0f);
  std::vector<FOV> fov_vector = {fov1, fov2};
  PolarPoint p_outside_azimuth(-1.f, 57.0f, 2.0f);
  PolarPoint p_outside_elevation(-60.0f, 30.0f, 2.0f);
  PolarPoint p_inside_fov1(25.0f, 27.0f, 2.0f);
  PolarPoint p_inside_fov2(25.0f, 70.0f, 2.0f);

  // WHEN: we check whether they are inside the FOV
  bool outside_azimuth = pointInsideFOV(fov_vector, p_outside_azimuth);
  bool outside_elevation = pointInsideFOV(fov_vector, p_outside_elevation);
  bool inside = pointInsideFOV(fov_vector, p_inside_fov1);
  bool inside2 = pointInsideFOV(fov_vector, p_inside_fov2);

  // THEN: they should lie in the expected region
  EXPECT_FALSE(outside_azimuth);
  EXPECT_FALSE(outside_elevation);
  EXPECT_TRUE(inside);
  EXPECT_TRUE(inside2);
}

TEST(PlannerFunctions, histogramIndexYawInsideFOV) {
  FOV fov(34.0f, 12.0f, 90.0f, 60.0f);
  PolarPoint fov_pol_low;
  fov_pol_low.e = 0;
  fov_pol_low.z = fov.yaw_deg - (90.f / 2.f);
  fov_pol_low.r = 2;
  Eigen::Vector2i index_low = polarToHistogramIndex(fov_pol_low, ALPHA_RES);
  PolarPoint fov_pol_high;
  fov_pol_high.e = 0;
  fov_pol_high.z = fov.yaw_deg + (90.f / 2.f);
  fov_pol_high.r = 2;
  Eigen::Vector2i index_high = polarToHistogramIndex(fov_pol_high, ALPHA_RES);

  for (int i = 0; i < 360 / ALPHA_RES; i++) {
    bool res = histogramIndexYawInsideFOV(fov, i, Eigen::Vector3f(1.f, 1.f, 5.f), 20.f);
    if (i >= index_low.x() && i <= index_high.x()) {
      EXPECT_TRUE(res);
    } else {
      EXPECT_FALSE(res);
    }
  }
}

TEST(PlannerFunctions, histogramIndexYawInsideFOVVector) {
  FOV fov1(34.0f, 12.0f, 45.0f, 60.0f);
  FOV fov2(80.0f, 12.0f, 45.0f, 60.0f);
  std::vector<FOV> fov_vector = {fov1, fov2};
  PolarPoint fov_pol_low1;
  fov_pol_low1.e = 0;
  fov_pol_low1.z = fov1.yaw_deg - (fov1.h_fov_deg / 2.f);
  fov_pol_low1.r = 2;
  Eigen::Vector2i index_low1 = polarToHistogramIndex(fov_pol_low1, ALPHA_RES);
  PolarPoint fov_pol_high1;
  fov_pol_high1.e = 0;
  fov_pol_high1.z = fov1.yaw_deg + (fov1.h_fov_deg / 2.f);
  fov_pol_high1.r = 2;
  Eigen::Vector2i index_high1 = polarToHistogramIndex(fov_pol_high1, ALPHA_RES);
  PolarPoint fov_pol_low2;
  fov_pol_low2.e = 0;
  fov_pol_low2.z = fov2.yaw_deg - (fov2.h_fov_deg / 2.f);
  fov_pol_low2.r = 2;
  Eigen::Vector2i index_low2 = polarToHistogramIndex(fov_pol_low2, ALPHA_RES);
  PolarPoint fov_pol_high2;
  fov_pol_high2.e = 0;
  fov_pol_high2.z = fov2.yaw_deg + (fov2.h_fov_deg / 2.f);
  fov_pol_high2.r = 2;
  Eigen::Vector2i index_high2 = polarToHistogramIndex(fov_pol_high2, ALPHA_RES);

  for (int i = 0; i < 360 / ALPHA_RES; i++) {
    bool res = histogramIndexYawInsideFOV(fov_vector, i, Eigen::Vector3f(1.f, 1.f, 5.f), 340.f);
    if ((i >= index_low1.x() && i <= index_high1.x()) || (i >= index_low2.x() && i < index_high2.x())) {
      EXPECT_TRUE(res);
    } else {
      EXPECT_FALSE(res);
    }
  }
}

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
  float angle_diff1 = indexAngleDifference(a1, b1);
  float angle_diff2 = indexAngleDifference(a2, b2);
  float angle_diff3 = indexAngleDifference(a3, b3);
  float angle_diff4 = indexAngleDifference(a4, b4);

  // THEN: the angle should be...
  EXPECT_FLOAT_EQ(0.f, angle_diff1);
  EXPECT_FLOAT_EQ(0.f, angle_diff2);
  EXPECT_FLOAT_EQ(0.f, angle_diff3);
  ASSERT_NEAR(0.99999f, angle_diff4, .00001f);
}

TEST(Common, azimuthAnglefromCartesian) {
  // GIVEN: two points
  const Eigen::Vector3f point_right(1.0f, 0.0f, 0.0f);
  const Eigen::Vector3f point_up(0.0f, 1.0f, 0.0f);
  const Eigen::Vector3f point_left(-1.0f, 0.0f, 0.0f);
  const Eigen::Vector3f point_down(0.0f, -1.0f, 0.0f);
  const Eigen::Vector3f point_q1(1.0f, 2.0f, 0.0f);
  const Eigen::Vector3f point_q2(-3.0f, 4.0f, 0.0f);
  const Eigen::Vector3f point_q3(-5.0f, -6.0f, 0.0f);
  const Eigen::Vector3f point_q4(7.0f, -8.0f, 0.0f);
  const Eigen::Vector3f origin(0.0f, 0.0f, 0.0f);

  // WHEN: calculating the azimuth angle between the two points
  float angle_right = cartesianToPolarHistogram(point_right, origin).z;
  float angle_up = cartesianToPolarHistogram(point_up, origin).z;
  float angle_left = cartesianToPolarHistogram(point_left, origin).z;
  float angle_down = cartesianToPolarHistogram(point_down, origin).z;
  float angle_undetermined = cartesianToPolarHistogram(origin, origin).z;
  float angle_q1 = cartesianToPolarHistogram(point_q1, origin).z;
  float angle_q2 = cartesianToPolarHistogram(point_q2, origin).z;
  float angle_q3 = cartesianToPolarHistogram(point_q3, origin).z;
  float angle_q4 = cartesianToPolarHistogram(point_q4, origin).z;
  float angle_non_zero_origin = cartesianToPolarHistogram(point_q1, point_q2).z;

  // THEN:  angle should be ..
  EXPECT_FLOAT_EQ(90.f, angle_right);
  EXPECT_FLOAT_EQ(0.f, angle_up);
  EXPECT_FLOAT_EQ(180.f, angle_down);
  EXPECT_FLOAT_EQ(-90.f, angle_left);
  EXPECT_FLOAT_EQ(0.f, angle_undetermined);
  EXPECT_FLOAT_EQ(26.565051f, angle_q1);
  EXPECT_FLOAT_EQ(-36.869897f, angle_q2);
  EXPECT_FLOAT_EQ(-140.194428f, angle_q3);
  EXPECT_FLOAT_EQ(138.814074f, angle_q4);
  EXPECT_FLOAT_EQ(116.565051f, angle_non_zero_origin);
}

TEST(Common, elevationAnglefromCartesian) {
  // GIVEN: two points
  const Eigen::Vector3f point_front(0.0f, 1.0f, 0.0f);
  const Eigen::Vector3f point_up(0.0f, 0.0f, 1.0f);
  const Eigen::Vector3f point_behind(0.0f, -1.0f, 0.0f);
  const Eigen::Vector3f point_down(0.0f, 0.0f, -1.0f);
  const Eigen::Vector3f point_q1(0.0f, 1.0f, 1.732050808f);
  const Eigen::Vector3f point_30(0.0f, 1.0f, 0.577350269f);
  const Eigen::Vector3f point_q2(0.0f, -3.0f, 4.0f);
  const Eigen::Vector3f point_q3(0.0f, -5.0f, -6.0f);
  const Eigen::Vector3f point_q4(0.0f, 7.0f, -8.0f);
  const Eigen::Vector3f origin(0.0f, 0.0f, 0.0f);

  // WHEN: we get the elevation angle between the two points
  const float angle_front = cartesianToPolarHistogram(point_front, origin).e;
  const float angle_up = cartesianToPolarHistogram(point_up, origin).e;
  const float angle_behind = cartesianToPolarHistogram(point_behind, origin).e;
  const float angle_down = cartesianToPolarHistogram(point_down, origin).e;
  const float angle_undetermined = cartesianToPolarHistogram(origin, origin).e;
  const float angle_q1 = cartesianToPolarHistogram(point_q1, origin).e;
  const float angle_30 = cartesianToPolarHistogram(point_30, origin).e;
  const float angle_q2 = cartesianToPolarHistogram(point_q2, origin).e;
  const float angle_q3 = cartesianToPolarHistogram(point_q3, origin).e;
  const float angle_q4 = cartesianToPolarHistogram(point_q4, origin).e;
  const float angle_non_zero_origin = cartesianToPolarHistogram(point_q4, point_q2).e;

  // THEN: angle should be ..
  EXPECT_FLOAT_EQ(0.f, angle_front);
  EXPECT_FLOAT_EQ(90.f, angle_up);
  EXPECT_FLOAT_EQ(0.f, angle_behind);
  EXPECT_FLOAT_EQ(-90.f, angle_down);
  EXPECT_FLOAT_EQ(0.f, angle_undetermined);
  EXPECT_FLOAT_EQ(60.f, angle_q1);
  EXPECT_FLOAT_EQ(30.f, angle_30);
  EXPECT_FLOAT_EQ(53.130110f, angle_q2);
  EXPECT_FLOAT_EQ(-50.194429f, angle_q3);
  EXPECT_FLOAT_EQ(-48.8140748f, angle_q4);
  EXPECT_FLOAT_EQ(-50.194428f, angle_non_zero_origin);
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
  PolarPoint p_pol_7(-90.0f, -180.00001f, 0.0f);

  const float resolution_1 = 3.f;
  const float resolution_2 = 12.f;

  // WHEN: we convert the polar point to a histogram index
  const Eigen::Vector2i index_1 = polarToHistogramIndex(p_pol_1, resolution_1);
  const Eigen::Vector2i index_2 = polarToHistogramIndex(p_pol_2, resolution_1);
  const Eigen::Vector2i index_3 = polarToHistogramIndex(p_pol_1, resolution_2);
  const Eigen::Vector2i index_4 = polarToHistogramIndex(p_pol_2, resolution_2);
  const Eigen::Vector2i index_5 = polarToHistogramIndex(p_pol_3, resolution_2);
  const Eigen::Vector2i index_6 = polarToHistogramIndex(p_pol_4, resolution_2);
  const Eigen::Vector2i index_7 = polarToHistogramIndex(p_pol_5, resolution_1);  // wrapped
  const Eigen::Vector2i index_8 = polarToHistogramIndex(p_pol_6, resolution_2);  // wrapped
  const Eigen::Vector2i index_9 = polarToHistogramIndex(p_pol_7, resolution_2);  // wrapped

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
  EXPECT_EQ(0, index_9.y());
  // azimuth angle
  EXPECT_EQ(60, index_1.x());
  EXPECT_EQ(71, index_2.x());
  EXPECT_EQ(15, index_3.x());
  EXPECT_EQ(17, index_4.x());
  EXPECT_EQ(0, index_5.x());
  EXPECT_EQ(0, index_6.x());
  EXPECT_EQ(66, index_7.x());
  EXPECT_EQ(22, index_8.x());
  EXPECT_EQ(29, index_9.x());
}

TEST(Common, polarHistogramToCartesian) {
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
    pos_out.push_back(polarHistogramToCartesian(p_pol, pos));
  }

  for (int i = 0; i < n; i++) {
    PolarPoint p_pol(e[i], z[i], radius[1]);
    pos_out.push_back(polarHistogramToCartesian(p_pol, pos));
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
  float radius = 2.168f;
  Eigen::Vector3f pos(4.21f, 2.34f, 0.19f);

  // WHEN: going through all valid polar coordinates and transform it to
  // cartesian and back again
  for (float e = -90.f; e <= 90.f; e = e + 3.f) {
    for (float z = -180.f; z <= 180.f; z = z + 6.f) {
      PolarPoint p_pol(e, z, radius);
      Eigen::Vector3f p_cartesian = polarHistogramToCartesian(p_pol, pos);

      PolarPoint p_pol_new = cartesianToPolarHistogram(p_cartesian, pos);

      // THEN: the resulting polar positions are expected to be the same as
      // before the conversion
      ASSERT_GE(p_pol_new.z, -180.f);
      ASSERT_LE(p_pol_new.z, 180.f);
      ASSERT_GE(p_pol_new.e, -90.f);
      ASSERT_LE(p_pol_new.e, 90.f);

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

TEST(Common, cartesianTopolarHistogramToCartesian) {
  // GIVEN: a current position
  Eigen::Vector3f pos(0.81f, 5.17f, 3.84f);

  // WHEN: going through some valid cartesian coordinates and transform it to
  // polar and back to cartesian
  for (float x = -5.f; x <= 5.f; x = x + 0.5f) {
    for (float y = -5.f; y <= 5.f; y = y + 0.6f) {
      for (float z = -5.f; z <= 5.f; z = z + 0.4f) {
        Eigen::Vector3f origin(x, y, z);
        PolarPoint p_pol = cartesianToPolarHistogram(origin, pos);
        // p_pol.r = (origin - pos).norm();
        Eigen::Vector3f p_cartesian = polarHistogramToCartesian(p_pol, pos);

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
  Eigen::Vector3f location(0.0f, 0.0f, 0.0f);
  Eigen::Vector3f next_pos1(1.0f, 1.0f, 0.0f);
  Eigen::Vector3f next_pos2(0.0f, 0.0f, 0.0f);

  // WHEN: we get the yaw between the two points
  float yaw1 = nextYaw(location, next_pos1);
  float yaw2 = nextYaw(location, next_pos2);

  // THEN: the angle in rad should be...
  EXPECT_NEAR(0.785398163f, yaw1, 0.00001f);
  EXPECT_NEAR(0.f, yaw2, 0.00001f);
}

TEST(Common, wrapAngle) {
  // GIVEN: an angle in rad
  float angle1 = 0.f;
  float angle2 = 30.f * M_PI_F / 180.f;
  float angle3 = 270.f * M_PI_F / 180.f;
  float angle4 = -90.f * M_PI_F / 180.f;
  float angle5 = -225.f * M_PI_F / 180.f;
  float angle6 = std::numeric_limits<float>::infinity();

  // WHEN: it is wrapped to the space (-PI; PI] space
  angle1 = wrapAngleToPlusMinusPI(angle1);
  angle2 = wrapAngleToPlusMinusPI(angle2);
  angle3 = wrapAngleToPlusMinusPI(angle3);
  angle4 = wrapAngleToPlusMinusPI(angle4);
  angle5 = wrapAngleToPlusMinusPI(angle5);
  angle6 = wrapAngleToPlusMinusPI(angle6);

  // THEN: the output angles shoudl be ..
  EXPECT_FLOAT_EQ(0.f, angle1);
  EXPECT_FLOAT_EQ(0.523599f, angle2);
  EXPECT_FLOAT_EQ(-1.570796f, angle3);
  EXPECT_FLOAT_EQ(-1.570796f, angle4);
  EXPECT_FLOAT_EQ(2.356194f, angle5);
  EXPECT_TRUE(std::isnan(angle6));
}

TEST(Common, getAngularVel) {
  // GIVEN: maximum and minimum velocity, slope, old velocity and time elapsed
  float desired_yaw1 = 0.f;
  float desired_yaw2 = 540.f * M_PI_F / 180.f;
  float curr_yaw1 = 0.f * M_PI_F / 180.f;
  float curr_yaw2 = -45.f * M_PI_F / 180.f;

  // WHEN: we get distance between the same points
  double angular_vel1 = getAngularVelocity(desired_yaw1, curr_yaw1);
  double angular_vel2 = getAngularVelocity(desired_yaw2, curr_yaw1);
  double angular_vel3 = getAngularVelocity(desired_yaw1, curr_yaw2);
  double angular_vel4 = getAngularVelocity(desired_yaw2, curr_yaw2);

  // THEN: the distance should be...

  EXPECT_FLOAT_EQ(0.0, angular_vel1);
  EXPECT_FLOAT_EQ(1.5707963, angular_vel2);
  EXPECT_FLOAT_EQ(0.392699, angular_vel3);
  EXPECT_FLOAT_EQ(-1.178097, angular_vel4);
}

TEST(Common, IndexPolarIndex) {
  // GIVEN:  some valid histogram indices and resolution
  int res = 6;

  for (int e_ind = 0; e_ind < 30; e_ind += 5) {
    for (int z_ind = 0; z_ind < 60; z_ind += 5) {
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
  PolarPoint p_pol_1(110.f, 0.f, rad);    // wrap  elevation with jump in azimuth
  PolarPoint p_pol_2(0.f, 200.f, rad);    // wrap azimuth
  PolarPoint p_pol_3(-180.f, 0.f, rad);   // wrap elevation with jump in azimuth
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

TEST(Common, removeNaNAndGetMaxima) {
  // GIVEN: two point clouds, one including NANs, one without
  pcl::PointCloud<pcl::PointXYZ> pc_no_nan, pc_with_nan;
  for (float x = -40.f; x <= 40.f; x += 1.f) {
    for (float y = -30.f; y <= 30.f; y += 1.f) {
      pcl::PointXYZ p(x, y, 15.f);
      pc_no_nan.push_back(p);
      pc_with_nan.push_back(p);
      pc_with_nan.push_back(pcl::PointXYZ(NAN, x, y));  // garbage point
    }
  }
  pc_with_nan.is_dense = false;
  pc_no_nan.is_dense = true;

  // WHEN: we filter these clouds
  pcl::PointCloud<pcl::PointXYZ> maxima_no_nan, maxima_with_nan;
  maxima_no_nan = removeNaNAndGetMaxima(pc_no_nan);
  maxima_with_nan = removeNaNAndGetMaxima(pc_with_nan);

  // THEN: we expect the clouds to not contain NANs, be dense and reflect
  // the maxima and minima
  EXPECT_EQ(pc_no_nan.size(), pc_with_nan.size());
  EXPECT_TRUE(pc_no_nan.is_dense);
  EXPECT_TRUE(pc_with_nan.is_dense);
  float min_x = 999.f, min_y = 999.f, min_z = 999.f, max_x = -999.f, max_y = -999.f, max_z = -999.f;

  for (auto p : maxima_no_nan) {
    min_x = std::min(p.x, min_x);
    min_y = std::min(p.y, min_y);
    min_z = std::min(p.z, min_z);
    max_x = std::max(p.x, max_x);
    max_y = std::max(p.y, max_y);
    max_z = std::max(p.z, max_z);
  }
  EXPECT_NEAR(-40.f, min_x, 1.0f);
  EXPECT_NEAR(-30.f, min_y, 1.0f);
  EXPECT_NEAR(15.f, min_z, 1.0f);
  EXPECT_NEAR(40.f, max_x, 1.0f);
  EXPECT_NEAR(30.f, max_y, 1.0f);
  EXPECT_NEAR(15.f, max_z, 1.0f);

  for (auto p : maxima_with_nan) {
    min_x = std::min(p.x, min_x);
    min_y = std::min(p.y, min_y);
    min_z = std::min(p.z, min_z);
    max_x = std::max(p.x, max_x);
    max_y = std::max(p.y, max_y);
    max_z = std::max(p.z, max_z);
  }
  EXPECT_NEAR(-40.f, min_x, 1.0f);
  EXPECT_NEAR(-30.f, min_y, 1.0f);
  EXPECT_NEAR(15.f, min_z, 1.0f);
  EXPECT_NEAR(40.f, max_x, 1.0f);
  EXPECT_NEAR(30.f, max_y, 1.0f);
  EXPECT_NEAR(15.f, min_z, 1.0f);
}

TEST(Common, isInWhichFOV) {
  // GIVEN: a three-camera setup with two overlapping FOV and one alone
  /**
            cam 3                     cam1    cam 2
                                             |--|--|
          |---|---|                |-----|-----|
  |--------------------------------------|------------------------------------|
  -180                             -30   0     +30                          +180
  **/
  std::vector<FOV> fov;
  fov.push_back(FOV(0.0f, 0.0f, 60.0f, 40.0f));     // camera one: forward-facing
  fov.push_back(FOV(35.0f, -1.0f, 15.0f, 30.0f));   // camera two: right-facing with overlaps with 1
  fov.push_back(FOV(-100.0f, 3.0f, 20.0f, 30.0f));  // camera three: left facing

  // WHEN: we sample points inside, outside and in the overlapping region
  int idx_inside_camera_1, idx_inside_camera_2, idx_inside_camera_3, idx_outside, idx_overlap = 0;
  bool retval_inside_1, retval_inside_2, retval_inside_3, retval_outside, retval_overlap = false;
  PolarPoint sample_inside_1(0.0f, -25.0f, 1.0f);
  PolarPoint sample_inside_2(0.0f, 36.0f, 1.0f);
  PolarPoint sample_inside_3(0.0f, -90.0f, 1.0f);
  PolarPoint sample_outside(0.0f, 150.0f, 1.0f);
  PolarPoint sample_overlap(0.0f, 29.0f, 1.0f);
  retval_inside_1 = isInWhichFOV(fov, sample_inside_1, idx_inside_camera_1);
  retval_inside_2 = isInWhichFOV(fov, sample_inside_2, idx_inside_camera_2);
  retval_inside_3 = isInWhichFOV(fov, sample_inside_3, idx_inside_camera_3);
  retval_outside = isInWhichFOV(fov, sample_outside, idx_outside);
  retval_overlap = isInWhichFOV(fov, sample_overlap, idx_overlap);

  // THEN: we expect the output to be giving us the corresponding camera
  EXPECT_TRUE(retval_inside_1);
  EXPECT_TRUE(retval_inside_2);
  EXPECT_TRUE(retval_inside_3);
  EXPECT_FALSE(retval_outside);
  EXPECT_FALSE(retval_overlap);
  EXPECT_EQ(0, idx_inside_camera_1);
  EXPECT_EQ(1, idx_inside_camera_2);
  EXPECT_EQ(2, idx_inside_camera_3);
  EXPECT_EQ(-1, idx_outside);
  EXPECT_EQ(-1, idx_overlap);
}

TEST(Common, isOnEdgeOfFOV) {
  // GIVEN: a three-camera setup with two overlapping FOV and one alone
  /**
            cam 3                     cam1    cam 2
                                             |--|--|
          |---|---|                |-----|-----|
  |--------------------------------------|------------------------------------|
  -180                             -30   0     +30                          +180
  **/
  std::vector<FOV> fov;
  fov.push_back(FOV(0.0f, 0.0f, 60.0f, 40.0f));     // camera one: forward-facing
  fov.push_back(FOV(35.0f, -1.0f, 15.0f, 30.0f));   // camera two: right-facing with overlaps with 1
  fov.push_back(FOV(-100.0f, 3.0f, 20.0f, 30.0f));  // camera three: left facing

  // WHEN: we sample points outside, inside but on edge, inside but not on edge,
  // inside in overlapping region
  bool left_cam_1, right_cam_1, left_cam_2, right_cam_2, left_cam_3, right_cam_3, overlap, outside = false;
  int idx_left_cam_1, idx_right_cam_1, idx_left_cam_2, idx_right_cam_2, idx_left_cam_3, idx_right_cam_3, idx_overlap,
      idx_outside;
  PolarPoint sample_left_cam_1(0.0f, -25.0f, 1.0f);
  PolarPoint sample_right_cam_1(0.0f, 0.2f, 1.0f);
  PolarPoint sample_left_cam_2(0.0f, 34.0f, 1.0f);
  PolarPoint sample_right_cam_2(0.0f, 40.0f, 1.0f);
  PolarPoint sample_left_cam_3(0.0f, -105.0f, 1.0f);
  PolarPoint sample_right_cam_3(0.0f, -92.0f, 1.0f);
  PolarPoint sample_overlap(0.0f, 8.0f, 1.0f);
  PolarPoint sample_outside(0.0f, 50.0f, 1.0f);

  left_cam_1 = isOnEdgeOfFOV(fov, sample_left_cam_1, idx_left_cam_1);
  right_cam_1 = isOnEdgeOfFOV(fov, sample_right_cam_1, idx_right_cam_1);
  left_cam_2 = isOnEdgeOfFOV(fov, sample_left_cam_2, idx_left_cam_2);
  right_cam_2 = isOnEdgeOfFOV(fov, sample_right_cam_2, idx_right_cam_2);
  left_cam_3 = isOnEdgeOfFOV(fov, sample_left_cam_3, idx_left_cam_3);
  right_cam_3 = isOnEdgeOfFOV(fov, sample_right_cam_3, idx_right_cam_3);
  overlap = isOnEdgeOfFOV(fov, sample_overlap, idx_overlap);
  outside = isOnEdgeOfFOV(fov, sample_outside, idx_outside);

  // THEN: we expect the output to reflect that
  EXPECT_TRUE(left_cam_1);
  EXPECT_FALSE(right_cam_1);
  EXPECT_FALSE(left_cam_2);
  EXPECT_TRUE(right_cam_2);
  EXPECT_TRUE(left_cam_3);
  EXPECT_TRUE(right_cam_3);
  EXPECT_FALSE(overlap);
  EXPECT_FALSE(outside);
  EXPECT_EQ(0, idx_left_cam_1);    // because 0 is the index
  EXPECT_EQ(-1, idx_right_cam_1);  // because not on edge
  EXPECT_EQ(-1, idx_left_cam_2);   // because not on edge
  EXPECT_EQ(1, idx_right_cam_2);   // because 1 is the index
  EXPECT_EQ(2, idx_left_cam_3);
  EXPECT_EQ(2, idx_right_cam_3);
  EXPECT_EQ(-1, idx_overlap);  // because not on edge
  EXPECT_EQ(-1, idx_outside);  // because not on edge
}

TEST(Common, scaleToFOV) {
  // GIVEN: a three-camera setup with two overlapping FOV and one alone
  /**
            cam 3                     cam1    cam 2
                                             |--|--|
          |---|---|                |-----|-----|
  |--------------------------------------|------------------------------------|
  -180                             -30   0     +30                          +180
  **/
  std::vector<FOV> fov;
  fov.push_back(FOV(0.0f, 0.0f, 60.0f, 40.0f));     // camera one: forward-facing
  fov.push_back(FOV(35.0f, -1.0f, 15.0f, 30.0f));   // camera two: right-facing with overlaps with 1
  fov.push_back(FOV(-100.0f, 3.0f, 20.0f, 30.0f));  // camera three: left facing

  // WHEN: we sample points outside, inside but on edge, inside but not on edge,
  // inside in overlapping region
  float left_cam_1, left_2_cam_1, right_cam_1, left_cam_2, right_cam_2, left_cam_3, right_cam_3, overlap,
      outside = false;
  int idx_left_cam_1, idx_right_cam_1, idx_left_cam_2, idx_right_cam_2, idx_left_cam_3, idx_right_cam_3, idx_overlap,
      idx_outside;
  PolarPoint sample_left_cam_1(0.0f, -25.0f, 1.0f);
  PolarPoint sample2_left_cam_1(0.0f, -26.0f, 1.0f);
  PolarPoint sample_right_cam_1(0.0f, 0.2f, 1.0f);
  PolarPoint sample_left_cam_2(0.0f, 34.0f, 1.0f);
  PolarPoint sample_right_cam_2(0.0f, 40.0f, 1.0f);
  PolarPoint sample_left_cam_3(0.0f, -105.0f, 1.0f);
  PolarPoint sample_right_cam_3(0.0f, -92.0f, 1.0f);
  PolarPoint sample_overlap(0.0f, 8.0f, 1.0f);
  PolarPoint sample_outside(0.0f, 50.0f, 1.0f);

  left_cam_1 = scaleToFOV(fov, sample_left_cam_1);
  left_2_cam_1 = scaleToFOV(fov, sample2_left_cam_1);
  right_cam_1 = scaleToFOV(fov, sample_right_cam_1);
  left_cam_2 = scaleToFOV(fov, sample_left_cam_2);
  right_cam_2 = scaleToFOV(fov, sample_right_cam_2);
  left_cam_3 = scaleToFOV(fov, sample_left_cam_3);
  right_cam_3 = scaleToFOV(fov, sample_right_cam_3);
  overlap = scaleToFOV(fov, sample_overlap);
  outside = scaleToFOV(fov, sample_outside);

  // THEN: we expect the output to reflect that
  EXPECT_LT(0.0f, left_cam_1);
  EXPECT_GT(1.0f, left_cam_1);
  EXPECT_LT(0.0f, left_2_cam_1);
  EXPECT_GT(1.0f, left_2_cam_1);
  EXPECT_GT(left_cam_1, left_2_cam_1);  // sample 2 is farther away from center
  EXPECT_FLOAT_EQ(1.0f, right_cam_1);
  EXPECT_FLOAT_EQ(1.0f, left_cam_2);
  EXPECT_GT(1.0f, right_cam_2);
  EXPECT_LT(0.0f, right_cam_2);
  EXPECT_LT(0.0f, left_cam_3);
  EXPECT_GT(1.0f, left_cam_3);
  EXPECT_LT(0.0f, right_cam_3);
  EXPECT_GT(1.0f, right_cam_3);
  EXPECT_FLOAT_EQ(1.0f, overlap);
  EXPECT_FLOAT_EQ(0.0f, outside);
}

TEST(Common, updateFOVFromMaxima) {
  // GIVEN: different point clouds indicating various FOV
  pcl::PointCloud<pcl::PointXYZ> front_h60_v45, right_h90_v30, back_h78_v45, front_elevated, empty, one, zenith, nadir;
  FOV fov_front_h60_v45, fov_right_h90_v30, fov_back_h78_v45, fov_front_elevated, fov_empty, fov_one, fov_zenith,
      fov_nadir;
  front_h60_v45.push_back(pcl::PointXYZ(1.0f, 0.57735026f, 0.47829262f));
  front_h60_v45.push_back(pcl::PointXYZ(1.0f, -0.57735026f, -0.47829262f));
  right_h90_v30.push_back(pcl::PointXYZ(1.0f, -1.0f, -0.37893738196));
  right_h90_v30.push_back(pcl::PointXYZ(-1.0f, -1.0f, 0.37893738196));
  back_h78_v45.push_back(pcl::PointXYZ(-1.0f, -0.80978403319f, 0.5329932637f));
  back_h78_v45.push_back(pcl::PointXYZ(-1.0f, 0.80978403319f, -0.5329932637f));
  front_elevated.push_back(pcl::PointXYZ(1.0f, 1.0f, 1.0f));
  front_elevated.push_back(pcl::PointXYZ(1.0f, -1.0f, 2.0f));
  one.push_back(pcl::PointXYZ(1.0f, 1.0f, 1.0f));
  zenith.push_back(pcl::PointXYZ(1.0f, 1.0f, 3.0f));
  zenith.push_back(pcl::PointXYZ(1.0f, -1.0f, 3.0f));
  zenith.push_back(pcl::PointXYZ(-1.0f, 1.0f, 3.0f));
  zenith.push_back(pcl::PointXYZ(-1.0f, -1.0f, 3.0f));
  nadir.push_back(pcl::PointXYZ(1.0f, 1.0f, -3.0f));
  nadir.push_back(pcl::PointXYZ(1.0f, -1.0f, -3.0f));
  nadir.push_back(pcl::PointXYZ(-1.0f, 1.0f, -3.0f));
  nadir.push_back(pcl::PointXYZ(-1.0f, -1.0f, -3.0f));

  // WHEN: we call the updateFOVFromMaxima
  updateFOVFromMaxima(fov_front_h60_v45, front_h60_v45);

  // WHEN: we make subsequent calls with smaller or empty point clouds
  updateFOVFromMaxima(fov_front_h60_v45, empty);

  // WHEN: we test different fov point clouds
  updateFOVFromMaxima(fov_right_h90_v30, right_h90_v30);
  updateFOVFromMaxima(fov_back_h78_v45, back_h78_v45);
  EXPECT_NO_THROW(updateFOVFromMaxima(fov_empty, empty));
  updateFOVFromMaxima(fov_front_elevated, front_elevated);
  EXPECT_NO_THROW(updateFOVFromMaxima(fov_one, one));
  EXPECT_NO_THROW(updateFOVFromMaxima(fov_zenith, zenith));
  EXPECT_NO_THROW(updateFOVFromMaxima(fov_nadir, nadir));

  // THEN: we expect the corresponding field of view
  EXPECT_NEAR(60.0f, fov_front_h60_v45.h_fov_deg, 1.0f);
  EXPECT_NEAR(45.0f, fov_front_h60_v45.v_fov_deg, 1.0f);
  EXPECT_NEAR(0.0f, fov_front_h60_v45.yaw_deg, 1.0f);
  EXPECT_NEAR(0.0f, fov_front_h60_v45.pitch_deg, 1.0f);

  EXPECT_NEAR(90.0f, fov_right_h90_v30.h_fov_deg, 1.0f);
  EXPECT_NEAR(30.0f, fov_right_h90_v30.v_fov_deg, 1.0f);
  EXPECT_NEAR(-90.0f, fov_right_h90_v30.yaw_deg, 1.0f);
  EXPECT_NEAR(0.0f, fov_right_h90_v30.pitch_deg, 1.0f);

  // THEN: the backward-looking can have yaw of + or - 180
  EXPECT_NEAR(78.0f, fov_back_h78_v45.h_fov_deg, 1.0f);
  EXPECT_NEAR(45.0f, fov_back_h78_v45.v_fov_deg, 1.0f);
  EXPECT_TRUE(180.0f - fov_back_h78_v45.yaw_deg <= FLT_EPSILON || -180.0f - fov_back_h78_v45.yaw_deg <= FLT_EPSILON);
  EXPECT_NEAR(0.0f, fov_back_h78_v45.pitch_deg, 1.0f);

  EXPECT_NEAR(0.0f, fov_empty.h_fov_deg, 1.0f);
  EXPECT_NEAR(0.0f, fov_empty.v_fov_deg, 1.0f);
  EXPECT_NEAR(0.0f, fov_empty.yaw_deg, 1.0f);
  EXPECT_NEAR(0.0f, fov_empty.pitch_deg, 1.0f);

  EXPECT_NEAR(90.0f, fov_front_elevated.h_fov_deg, 1.0f);
  EXPECT_NEAR(19.47121f, fov_front_elevated.v_fov_deg, 1.0f);
  EXPECT_NEAR(0.0f, fov_front_elevated.yaw_deg, 1.0f);
  EXPECT_NEAR(-45.0f, fov_front_elevated.pitch_deg, 1.0f);

  EXPECT_NEAR(0.0f, fov_one.h_fov_deg, 1.0f);
  EXPECT_NEAR(0.0f, fov_one.v_fov_deg, 1.0f);
  EXPECT_NEAR(0.0f, fov_one.yaw_deg, 1.0f);
  EXPECT_NEAR(0.0f, fov_one.pitch_deg, 1.0f);
  /** these can currently not work, as long as it doesn't throw
    EXPECT_NEAR(360.0f, fov_zenith.h_fov_deg, 1.0f);
    EXPECT_NEAR(36.8698976f, fov_zenith.v_fov_deg, 1.0f);
    EXPECT_NEAR(0.0f, fov_zenith.yaw_deg, 1.0f);
    EXPECT_NEAR(90.0f, fov_zenith.pitch_deg, 1.0f);

    EXPECT_NEAR(360.0f, fov_nadir.h_fov_deg, 1.0f);
    EXPECT_NEAR(36.8698976f, fov_nadir.v_fov_deg, 1.0f);
    EXPECT_NEAR(0.0f, fov_nadir.yaw_deg, 1.0f);
    EXPECT_NEAR(-90.0f, fov_nadir.pitch_deg, 1.0f);
    **/
}

TEST(Common, transformToTrajectory) {
  ros::Time::init();
  geometry_msgs::PoseStamped position_sp;
  geometry_msgs::Twist velocity_sp;
  mavros_msgs::Trajectory trajectory_message;
  position_sp.pose.position.x = 1.3;
  position_sp.pose.position.y = 2.7;
  position_sp.pose.position.z = 5.0;
  position_sp.pose.orientation.x = 0.0;
  position_sp.pose.orientation.y = 0.0;
  position_sp.pose.orientation.z = 0.0;
  position_sp.pose.orientation.w = 1.0;

  velocity_sp.linear.x = NAN;
  velocity_sp.linear.y = NAN;
  velocity_sp.linear.z = 0.8;

  transformToTrajectory(trajectory_message, position_sp, velocity_sp);

  ASSERT_FLOAT_EQ(position_sp.pose.position.x, trajectory_message.point_1.position.x);
  ASSERT_FLOAT_EQ(position_sp.pose.position.y, trajectory_message.point_1.position.y);
  ASSERT_FLOAT_EQ(position_sp.pose.position.z, trajectory_message.point_1.position.z);
  ASSERT_FALSE(std::isfinite(velocity_sp.linear.x));
  ASSERT_FALSE(std::isfinite(velocity_sp.linear.y));
  ASSERT_FLOAT_EQ(velocity_sp.linear.z, trajectory_message.point_1.velocity.z);
  ASSERT_FLOAT_EQ(0.f, trajectory_message.point_1.yaw_rate);
  ASSERT_FLOAT_EQ(0.f, trajectory_message.point_1.yaw);
  ASSERT_FALSE(std::isfinite(trajectory_message.point_1.acceleration_or_force.x));
  ASSERT_FALSE(std::isfinite(trajectory_message.point_1.acceleration_or_force.y));
  ASSERT_FALSE(std::isfinite(trajectory_message.point_1.acceleration_or_force.z));

  ASSERT_TRUE(trajectory_message.point_valid[0]);

  ASSERT_FALSE(std::isfinite(trajectory_message.point_2.position.x));
  ASSERT_FALSE(std::isfinite(trajectory_message.point_2.position.y));
  ASSERT_FALSE(std::isfinite(trajectory_message.point_2.position.z));
  ASSERT_FALSE(std::isfinite(trajectory_message.point_2.velocity.x));
  ASSERT_FALSE(std::isfinite(trajectory_message.point_2.velocity.y));
  ASSERT_FALSE(std::isfinite(trajectory_message.point_2.velocity.z));
  ASSERT_FALSE(std::isfinite(trajectory_message.point_2.acceleration_or_force.x));
  ASSERT_FALSE(std::isfinite(trajectory_message.point_2.acceleration_or_force.y));
  ASSERT_FALSE(std::isfinite(trajectory_message.point_2.acceleration_or_force.z));
  ASSERT_FALSE(std::isfinite(trajectory_message.point_2.yaw));
  ASSERT_FALSE(std::isfinite(trajectory_message.point_2.yaw_rate));
  ASSERT_FALSE(trajectory_message.point_valid[1]);
}

TEST(Common, angleDifference) {
  float res1 = angleDifference(0.0f, 349.0f);
  ASSERT_FLOAT_EQ(11.f, res1);
  float res2 = angleDifference(179.8f, 0.f);
  ASSERT_FLOAT_EQ(179.8f, res2);
  float res3 = angleDifference(0.0f, 180.1f);
  ASSERT_FLOAT_EQ(179.9f, res3);
}

TEST(Common, getYawFromQuaternion) {
  Eigen::Quaternionf q = Eigen::Quaternionf(1.f, 0.f, 0.f, 0.f);
  float yaw1 = getYawFromQuaternion(q);
  ASSERT_FLOAT_EQ(0.f, yaw1);

  q = Eigen::Quaternionf(0.691718f, 0.311979f, 0.419491f, 0.498219f);
  float yaw2 = getYawFromQuaternion(q);
  ASSERT_FLOAT_EQ(80.942032f, yaw2);
}

TEST(Common, getPitchFromQuaternion) {
  float roll = 0.f;
  float pitch = 0.f;
  float yaw = 0.0f;
  Eigen::Quaternionf q = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()) *
                         Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
                         Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
  float pitch1 = getPitchFromQuaternion(q);
  ASSERT_FLOAT_EQ(0.f, pitch1);

  q = Eigen::Quaternionf(0.691718f, 0.311979f, 0.419491f, 0.498219f);
  float pitch2 = getPitchFromQuaternion(q);
  ASSERT_FLOAT_EQ(15.632803f, pitch2);
}

TEST(Common, createPoseMsg) {
  Eigen::Vector3f out_wp;
  Eigen::Quaternionf out_q;
  Eigen::Vector3f in = Eigen::Vector3f(1.2f, 5.5f, 2.3f);
  float yaw = 0.0f;
  createPoseMsg(out_wp, out_q, in, yaw);

  ASSERT_FLOAT_EQ(in.x(), out_wp.x());
  ASSERT_FLOAT_EQ(in.y(), out_wp.y());
  ASSERT_FLOAT_EQ(in.z(), out_wp.z());
  ASSERT_FLOAT_EQ(0.0f, out_q.x());
  ASSERT_FLOAT_EQ(0.0f, out_q.y());
  ASSERT_FLOAT_EQ(0.0f, out_q.z());
  ASSERT_FLOAT_EQ(1.0f, out_q.w());
}

TEST(Common, polarFCUToCartesian) {
  PolarPoint polar_p = cartesianToPolarFCU(Eigen::Vector3f(1.f, 1.f, 5.f), Eigen::Vector3f(0.0f, 0.0f, 5.0f));
  Eigen::Vector3f cartesian_p = polarFCUToCartesian(polar_p, Eigen::Vector3f(0.0f, 0.0f, 5.0f));
  ASSERT_FLOAT_EQ(1.f, cartesian_p.x());
  ASSERT_FLOAT_EQ(1.f, cartesian_p.y());
  ASSERT_FLOAT_EQ(5.f, cartesian_p.z());

  polar_p = cartesianToPolarFCU(Eigen::Vector3f(2.4f, 3.1f, 5.f), Eigen::Vector3f(0.2f, -0.4f, 5.0f));
  cartesian_p = polarFCUToCartesian(polar_p, Eigen::Vector3f(0.2f, -0.4f, 5.0f));
  ASSERT_FLOAT_EQ(2.4f, cartesian_p.x());
  ASSERT_FLOAT_EQ(3.1f, cartesian_p.y());
  ASSERT_FLOAT_EQ(5.f, cartesian_p.z());
}
