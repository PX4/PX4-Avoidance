#include <gtest/gtest.h>

#include "../src/nodes/common.h"

using namespace avoidance;

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
