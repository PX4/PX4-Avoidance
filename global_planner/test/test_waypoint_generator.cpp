#include <gtest/gtest.h>

#include "../include/global_planner/waypoint_generator.h"
#include "avoidance/common.h"

using namespace global_planner;

class WaypointGeneratorTests : public ::testing::Test, public WaypointGenerator {
 public:
  avoidanceOutput avoidance_output;
  Eigen::Vector3f position;
  Eigen::Quaternionf q;
  Eigen::Vector3f goal;
  Eigen::Vector3f prev_goal;
  Eigen::Vector3f velocity;
  bool is_land_waypoint;
  bool is_takeoff_waypoint;
  Eigen::Vector3f desired_velocity;

  bool stay = false;
  bool is_airborne = true;
  ros::Time time = ros::Time(0.33);

};

TEST_F(WaypointGeneratorTests, getSetpointFromPath) {
    const std::vector<Eigen::Vector3f> path;
    const ros::Time path_generation_time;
    float velocity;
    Eigen::Vector3f setpoint;
    bool result;
    
    result = getSetpointFromPath(path, path_generation_time, velocity, setpoint);

    EXPECT_EQ(result, false);
}