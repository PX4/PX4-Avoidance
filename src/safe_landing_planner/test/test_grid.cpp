#include <gtest/gtest.h>

#include "../include/safe_landing_planner/grid.hpp"

using namespace avoidance;

TEST(GridTest, gridCellSize) {
  Grid grid = Grid(6.f, 2.f);
  Eigen::Vector3f pos(1.2f, 3.4f, 2.f);
  grid.setFilterLimits(pos);
  Eigen::Vector2f limit_min, limit_max;
  grid.getGridLimits(limit_min, limit_max);

  EXPECT_FLOAT_EQ(-1.8f, limit_min.x());
  EXPECT_FLOAT_EQ(0.4f, limit_min.y());
  EXPECT_FLOAT_EQ(4.2f, limit_max.x());
  EXPECT_FLOAT_EQ(6.4f, limit_max.y());
}

TEST(GridTest, gridCellSize2) {
  Grid grid = Grid(12.f, 3.f);
  Eigen::Vector3f pos(-3.2f, 5.4f, 2.f);
  grid.setFilterLimits(pos);
  Eigen::Vector2f limit_min, limit_max;
  grid.getGridLimits(limit_min, limit_max);

  EXPECT_FLOAT_EQ(-9.2f, limit_min.x());
  EXPECT_FLOAT_EQ(-0.6f, limit_min.y());
  EXPECT_FLOAT_EQ(2.8f, limit_max.x());
  EXPECT_FLOAT_EQ(11.4f, limit_max.y());
}
