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


TEST(Common, L2Dfrom2points) {
  // GIVEN: two points
  geometry_msgs::Point position = createPoint(1.0d, 1.0d, 0.0d);

  pcl::PointXYZ xyz;
  xyz.x = 0.0d;
  xyz.y = 7.0d;
  xyz.z = -8.0d;
  // WHEN: we get the distance between the same points
  float dist = computeL2Dist(position, xyz);

  // THEN: the distance should be...
  EXPECT_NEAR(10.049875621, dist, 0.00001);
}

TEST(Common, distance3DCartesian) {
  // GIVEN: two points
  geometry_msgs::Point a = createPoint(1.0d, 1.0d, 0.0d);
  geometry_msgs::Point b = createPoint(0.0d, 7.0d, 8.0d);

  // WHEN: we get the distance between the same points
  float dist = distance3DCartesian(a, b) ;
  // THEN: the distance should be...
  EXPECT_NEAR(10.049875621, dist, 0.00001);
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


TEST(Common, azimuthAnglefromCartesian){
  //GIVEN: two points
  const geometry_msgs::Point point_right = createPoint(1.0d, 0.0d, 0.0d);
  const geometry_msgs::Point point_up = createPoint(0.0d, 1.0d, 0.0d);
  const geometry_msgs::Point point_left = createPoint(-1.0d, 0.0d, 0.0d);
  const geometry_msgs::Point point_down = createPoint(0.0d, -1.0d, 0.0d);
  const geometry_msgs::Point point_q1 = createPoint(1.0d, 2.0d, 0.0d);
  const geometry_msgs::Point point_q2 = createPoint(-3.0d, 4.0d, 0.0d);
  const geometry_msgs::Point point_q3 = createPoint(-5.0d, -6.0d, 0.0d);
  const geometry_msgs::Point point_q4 = createPoint(7.0d, -8.0d, 0.0d);
  const geometry_msgs::Point origin = createPoint(0.0d, 0.0d, 0.0d);

  //WHEN: calculating the azimuth angle between the two points
  float angle_right = azimuthAnglefromCartesian(point_right, origin);
  float angle_up = azimuthAnglefromCartesian(point_up, origin);
  float angle_left = azimuthAnglefromCartesian(point_left, origin);
  float angle_down = azimuthAnglefromCartesian(point_down, origin);
  float angle_undetermined = azimuthAnglefromCartesian(origin, origin);
  float angle_q1 = azimuthAnglefromCartesian(point_q1, origin);
  float angle_q2 = azimuthAnglefromCartesian(point_q2, origin);
  float angle_q3 = azimuthAnglefromCartesian(point_q3, origin);
  float angle_q4 = azimuthAnglefromCartesian(point_q4, origin);
  float angle_non_zero_origin = azimuthAnglefromCartesian(point_q1, point_q2);

  //THEN:  angle should be ..
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

TEST(Common, elevationAnglefromCartesian){
  //GIVEN: two points
  const geometry_msgs::Point point_front = createPoint(0.0d, 1.0d, 0.0d);
  const geometry_msgs::Point point_up = createPoint(0.0d, 0.0d, 1.0d);
  const geometry_msgs::Point point_behind = createPoint(0.0d, -1.0d, 0.0d);
  const geometry_msgs::Point point_down = createPoint(0.0d, 0.0d, -1.0d);
  const geometry_msgs::Point point_q1 = createPoint(0.0d, 1.0d, 1.732050808d);
  const geometry_msgs::Point point_30 = createPoint(0.0d, 1.0d, 0.577350269d);
  const geometry_msgs::Point point_q2 = createPoint(0.0d, -3.0d, 4.0d);
  const geometry_msgs::Point point_q3 = createPoint(0.0d, -5.0d, -6.0d);
  const geometry_msgs::Point point_q4 = createPoint(0.0d, 7.0d, -8.0d);
  const geometry_msgs::Point origin = createPoint(0.0d, 0.0d, 0.0d);

  //WHEN: we get the elevation angle between the two points
  const float angle_front = elevationAnglefromCartesian(point_front, origin);
  const float angle_up = elevationAnglefromCartesian(point_up, origin);
  const float angle_behind = elevationAnglefromCartesian(point_behind, origin);
  const float angle_down = elevationAnglefromCartesian(point_down, origin);
  const float angle_undetermined = elevationAnglefromCartesian(origin, origin);
  const float angle_q1 = elevationAnglefromCartesian(point_q1, origin);
  const float angle_30 = elevationAnglefromCartesian(point_30, origin);
  const float angle_q2 = elevationAnglefromCartesian(point_q2, origin);
  const float angle_q3 = elevationAnglefromCartesian(point_q3, origin);
  const float angle_q4 = elevationAnglefromCartesian(point_q4, origin);
  const float angle_non_zero_origin = elevationAnglefromCartesian(point_q4, point_q2);

  //THEN: angle should be ..
  EXPECT_FLOAT_EQ(0.f, angle_front);
  EXPECT_FLOAT_EQ(0.f, angle_up);
  EXPECT_FLOAT_EQ(0.f, angle_behind);
  EXPECT_FLOAT_EQ(0.f, angle_down);
  EXPECT_FLOAT_EQ(0.f, angle_undetermined);
  EXPECT_FLOAT_EQ(60.f, angle_q1);
  EXPECT_FLOAT_EQ(30.f, angle_30);
  EXPECT_FLOAT_EQ(53.130110, angle_q2);
  EXPECT_FLOAT_EQ(-50.194429, angle_q3);
  EXPECT_FLOAT_EQ(-48.8140748, angle_q4);
  EXPECT_FLOAT_EQ(-50.194428, angle_non_zero_origin);
}

TEST(Common, elevationAngletoIndex){
  //GIVEN: the elevation angle of a point and the histogram resolution 
  const float elevation_1 = 0.f;
  const float elevation_2 = 34.f;
  const float elevation_3 = 90.f;
  const float elevation_4 = -90.f;
  const float resolution_1 = 3.f;
  const float resolution_2 = 12.f;
  const float elevation_invalid_1 = 94.f;
  const float elevation_invalid_2 = -999.f;
  const float resolution_invalid_1 = 0.f;
  const float resolution_invalid_2 = -1.f;

  //WHEN: we convert the elevation angle to a histogram index
  const int index_1 = elevationAngletoIndex(elevation_1, resolution_1);
  const int index_2 = elevationAngletoIndex(elevation_2, resolution_1);
  const int index_3 = elevationAngletoIndex(elevation_1, resolution_2);
  const int index_4 = elevationAngletoIndex(elevation_2, resolution_2);
  const int index_5 = elevationAngletoIndex(elevation_3, resolution_2);
  const int index_6 = elevationAngletoIndex(elevation_4, resolution_2);
  const int index_invalid_1 = elevationAngletoIndex(elevation_invalid_1, resolution_1);
  const int index_invalid_2 = elevationAngletoIndex(elevation_invalid_2, resolution_1);
  const int index_invalid_3 = elevationAngletoIndex(elevation_1, resolution_invalid_1);
  const int index_invalid_4 = elevationAngletoIndex(elevation_1, resolution_invalid_2);

  //THEN: the vertical histogram index should be ..
  EXPECT_EQ(30, index_1);
  EXPECT_EQ(41, index_2);
  EXPECT_EQ(7, index_3);
  EXPECT_EQ(10, index_4);
  EXPECT_EQ(14, index_5);
  EXPECT_EQ(0, index_6);
  EXPECT_EQ(0, index_invalid_1);
  EXPECT_EQ(0, index_invalid_2);
  EXPECT_EQ(0, index_invalid_3);
  EXPECT_EQ(0, index_invalid_4);

}
TEST(Common, azimuthAngletoIndex){
  //GIVEN: the azimuth angle of a point and the histogram resolution
  const float elevation_1 = 0.f;
  const float elevation_2 = 34.f;
  const float elevation_3 = 180.f;
  const float elevation_4 = 179.f;
  const float elevation_5 = -180.f;
  const float resolution_1 = 3.f;
  const float resolution_2 = 12.f;
  const float elevation_invalid_1 = 194.f;
  const float elevation_invalid_2 = -999.f;
  const float resolution_invalid_1 = 0.f;
  const float resolution_invalid_2 = -1.f;

  //WHEN: we convert the azimuth angle to a histogram index
  const int index_1 = azimuthAngletoIndex(elevation_1, resolution_1);
  const int index_2 = azimuthAngletoIndex(elevation_2, resolution_1);
  const int index_3 = azimuthAngletoIndex(elevation_1, resolution_2);
  const int index_4 = azimuthAngletoIndex(elevation_2, resolution_2);
  const int index_5 = azimuthAngletoIndex(elevation_3, resolution_2);
  const int index_6 = azimuthAngletoIndex(elevation_4, resolution_2);
  const int index_7 = azimuthAngletoIndex(elevation_5, resolution_2);
  const int index_invalid_1 = azimuthAngletoIndex(elevation_invalid_1, resolution_1);
  const int index_invalid_2 = azimuthAngletoIndex(elevation_invalid_2, resolution_1);
  const int index_invalid_3 = azimuthAngletoIndex(elevation_1, resolution_invalid_1);
  const int index_invalid_4 = azimuthAngletoIndex(elevation_1, resolution_invalid_2);

  //THEN: the horizontal histogram index should be ..
  EXPECT_EQ(60, index_1);
  EXPECT_EQ(71, index_2);
  EXPECT_EQ(15, index_3);
  EXPECT_EQ(17, index_4);
  EXPECT_EQ(0, index_5);
  EXPECT_EQ(29, index_6);
  EXPECT_EQ(0, index_7);
  EXPECT_EQ(0, index_invalid_1);
  EXPECT_EQ(0, index_invalid_2);
  EXPECT_EQ(0, index_invalid_3);
  EXPECT_EQ(0, index_invalid_4);

}

TEST(Common, fromPolarToCartesian){

//GIVEN: the elevation angle, azimuth angle, a radius and the position
std::vector<float> e = {-90.f, -90.f, 90.f, 0.f, 45.f};  //[-90, 90]
std::vector<float> z = {-180.f, -90.f, 179.f, 0.f, 45.f}; //[-180, 180]

std::vector<double> radius = {0.f, 2.f};

geometry_msgs::Point pos;
pos.x = 0.f;
pos.y = 0.f;
pos.z = 0.f;

std::vector<geometry_msgs::Point> pos_out;

//WHEN: converting the point in polar CS to cartesian CS

for(int i = 0; i<e.size(); i++){
     pos_out.push_back(fromPolarToCartesian(e[i], z[3], radius[0], pos));
}

for(int i = 0; i<e.size(); i++){  
       pos_out.push_back(fromPolarToCartesian(e[i], z[i], radius[1], pos));
}


//THEN: the cartesian coordinates are

for(int i = 0; i<e.size(); i++){
    EXPECT_FLOAT_EQ(0.f, pos_out[i].x);
    EXPECT_FLOAT_EQ(0.f, pos_out[i].y);
    EXPECT_FLOAT_EQ(0.f, pos_out[i].z);  
}


EXPECT_NEAR(-0.f, pos_out[5].x, 0.00001);
EXPECT_NEAR(-0.f, pos_out[5].y, 0.00001);
EXPECT_FLOAT_EQ(-2.f, pos_out[5].z);  

EXPECT_NEAR(-0.f, pos_out[6].x, 0.00001);
EXPECT_NEAR(0.f, pos_out[6].y, 0.00001);
EXPECT_FLOAT_EQ(-2.f, pos_out[6].z);  

EXPECT_NEAR(0.f, pos_out[7].x, 0.00001);
EXPECT_NEAR(-0.f, pos_out[7].y, 0.00001);
EXPECT_FLOAT_EQ(2.f, pos_out[7].z);  

EXPECT_NEAR(0.f, pos_out[8].x, 0.00001);
EXPECT_NEAR(2.f, pos_out[8].y, 0.00001);
EXPECT_NEAR(0.f, pos_out[8].z, 0.00001);

EXPECT_NEAR(1.f, pos_out[9].x, 0.00001);
EXPECT_NEAR(1.f, pos_out[9].y, 0.00001);
EXPECT_NEAR(1.414213562, pos_out[9].z, 0.00001);
}


TEST(Common, nextYaw) {
  // GIVEN: two points
  geometry_msgs::PoseStamped location;
  location.pose.position.x = 0;
  location.pose.position.y = 0;
  location.pose.position.z = 0;
  geometry_msgs::Point next_pos1= createPoint(1.0d, 1.0d, 0.0d);
  geometry_msgs::Point next_pos2= createPoint(0.0d, 0.0d, 0.0d);

 // WHEN: we get the yaw between the two points
  double yaw1 = nextYaw(location, next_pos1);
  double yaw2 = nextYaw(location, next_pos2);
  
  // THEN: the angle in rad should be...
  EXPECT_NEAR(0.785398163, yaw1, 0.00001);
  EXPECT_NEAR(0.f, yaw2, 0.00001);
}



TEST(Common, normalizePoint) {
  // GIVEN: a point
  geometry_msgs::Point next_pos1= createPoint(1.0d, 1.0d, 0.0d);
  geometry_msgs::Point next_pos2= createPoint(0.0d, 0.0d, 0.0d);
  // WHEN: normalize the point
  normalize(next_pos1);
  normalize(next_pos2);
  // THEN: the point should be...
  EXPECT_FLOAT_EQ(0.707106781, next_pos1.x);
  EXPECT_FLOAT_EQ(0.707106781, next_pos1.y);
  EXPECT_FLOAT_EQ(0.f, next_pos1.z);
  EXPECT_FLOAT_EQ(0.f, next_pos2.x);
  EXPECT_FLOAT_EQ(0.f, next_pos2.y);
  EXPECT_FLOAT_EQ(0.f, next_pos2.z);
}


TEST(Common, speedCalc) {
  // GIVEN: maximum and minimum velocity, slope, old velocity and time elapsed
  double max_vel1 = 0.d; 
  double max_vel2 = 10.d;
  double min_vel1 = 0.d;
  double min_vel2 = 2.d;
  double slope = 1.d; // always one
  double v_old1 = 0.d;
  double v_old2 = 10.d;
  double elapsed = 2.d;


  // WHEN: we get distance between the same points
  double speed1 = velocityLinear(max_vel2, slope, v_old1, elapsed);
  double speed2 = velocityLinear(max_vel1, slope, v_old2, elapsed);
  double speed3 = velocityLinear(max_vel2, slope, v_old2, elapsed);

  // THEN: the distance should be...
  EXPECT_FLOAT_EQ(2.f, speed1);
  EXPECT_FLOAT_EQ(0.f, speed2);
  EXPECT_FLOAT_EQ(10.f, speed3);
}
