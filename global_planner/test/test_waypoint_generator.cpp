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
    double time;
    float velocity = 5.0;
    Eigen::Vector3f setpoint;
    bool result;

    //Empty path
    path.clear();
    result = getSetpointFromPath(path, time, velocity, setpoint);
    EXPECT_EQ(result, false);

    //Pathsize=1 TODO: THere is a bug
    time = 0.0;
    path.push_back(Eigen::Vector3f(0.5, 1.0, 2.0));
    result = getSetpointFromPath(path, time, velocity, setpoint);
    EXPECT_EQ(result, false);


    time = 0.0;
    path.push_back(Eigen::Vector3f(0.5, 1.0, 2.0));
    result = getSetpointFromPath(path, time, velocity, setpoint);

    EXPECT_EQ(result, true);
    EXPECT_EQ(path.size(), 2);
    EXPECT_EQ(result, true);
    EXEPCT_FLOAT_EQ(path[0].x(), setpoint.x());
    EXEPCT_FLOAT_EQ(path[1].y(), setpoint.y());
    EXEPCT_FLOAT_EQ(path[2].z(), setpoint.z());

}