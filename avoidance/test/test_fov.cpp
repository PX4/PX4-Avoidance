#include <gtest/gtest.h>
#include <limits>
#include "avoidance/fov.h"

using namespace avoidance;

TEST(Fov, isVisible) {
  // GIVEN: different fov objects
  FOV fov_front_h60_v45(60, 30), fov_right_h90_v30(60, 30),
      fov_back_h78_v45(60, 30), fov_front_elevated(60, 30);

  // WHEN: we call the updateWithPoint
  fov_front_h60_v45.updateWithPoint(1.0f, 0.57735026f, 0.47829262f);
  fov_front_h60_v45.updateWithPoint(1.0f, -0.57735026f, -0.47829262f);
  fov_right_h90_v30.updateWithPoint(1.0f, -1.0f, -0.37893738196);
  fov_right_h90_v30.updateWithPoint(-1.0f, -1.0f, 0.37893738196);
  fov_back_h78_v45.updateWithPoint(-1.0f, -0.80978403319f, 0.5329932637f);
  fov_back_h78_v45.updateWithPoint(-1.0f, 0.80978403319f, -0.5329932637f);
  fov_front_elevated.updateWithPoint(1.0f, 1.0f, 1.0f);
  fov_front_elevated.updateWithPoint(1.0f, -1.0f, 2.0f);

  // THEN:
  EXPECT_TRUE(fov_front_h60_v45.isVisible(0.0f, 0.0f));
  EXPECT_TRUE(fov_front_h60_v45.isVisible(25.0f, 0.0f));
  EXPECT_TRUE(fov_front_h60_v45.isVisible(-25.0f, 10.0f));
  EXPECT_TRUE(fov_front_h60_v45.isVisible(-365.0f, 10.0f));
  EXPECT_TRUE(fov_front_h60_v45.isVisible(180.0f, 100.0f));
  EXPECT_TRUE(fov_front_h60_v45.isVisible(0.0f, 360.0f));
  EXPECT_FALSE(fov_front_h60_v45.isVisible(0.0f, 190.0f));
  EXPECT_FALSE(fov_front_h60_v45.isVisible(40.0f, 0.0f));
  EXPECT_FALSE(fov_front_h60_v45.isVisible(10.0f, 50.0f));
  EXPECT_FALSE(fov_front_h60_v45.isVisible(10.0f, -90.0f));
}

TEST(Fov, scaleToFOV) {
  // GIVEN: different fov objects
  FOV fov_front_h60_v45(60, 30), fov_right_h90_v30(60, 30),
      fov_back_h78_v45(60, 30), fov_front_elevated(60, 30);

  // WHEN: we call the updateWithPoint
  fov_front_h60_v45.updateWithPoint(1.0f, 0.57735026f, 0.47829262f);
  fov_front_h60_v45.updateWithPoint(1.0f, -0.57735026f, -0.47829262f);
  fov_right_h90_v30.updateWithPoint(1.0f, -1.0f, -0.37893738196);
  fov_right_h90_v30.updateWithPoint(-1.0f, -1.0f, 0.37893738196);
  fov_back_h78_v45.updateWithPoint(-1.0f, -0.80978403319f, 0.5329932637f);
  fov_back_h78_v45.updateWithPoint(-1.0f, 0.80978403319f, -0.5329932637f);
  fov_front_elevated.updateWithPoint(1.0f, 1.0f, 1.0f);
  fov_front_elevated.updateWithPoint(1.0f, -1.0f, 2.0f);

  // failing test because not implemented!
  EXPECT_TRUE(1.0f == 2.0f);
}

TEST(Common, updateWithPoint) {
  // failing test because not implemented!
  EXPECT_TRUE(1.0f == 2.0f);

  // GIVEN: different fov objects
  FOV fov_front_h60_v45(60, 30), fov_right_h90_v30(60, 30),
      fov_back_h78_v45(60, 30), fov_front_elevated(60, 30);

  // WHEN: we call the updateWithPoint
  fov_front_h60_v45.updateWithPoint(1.0f, 0.57735026f, 0.47829262f);
  fov_front_h60_v45.updateWithPoint(1.0f, -0.57735026f, -0.47829262f);
  fov_right_h90_v30.updateWithPoint(1.0f, -1.0f, -0.37893738196);
  fov_right_h90_v30.updateWithPoint(-1.0f, -1.0f, 0.37893738196);
  fov_back_h78_v45.updateWithPoint(-1.0f, -0.80978403319f, 0.5329932637f);
  fov_back_h78_v45.updateWithPoint(-1.0f, 0.80978403319f, -0.5329932637f);
  fov_front_elevated.updateWithPoint(1.0f, 1.0f, 1.0f);
  fov_front_elevated.updateWithPoint(1.0f, -1.0f, 2.0f);
}
