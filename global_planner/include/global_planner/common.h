#ifndef GLOBAL_PLANNER_COMMON_H_
#define GLOBAL_PLANNER_COMMON_H_
#define M_PI 3.1415926535897932384626433832
#define EARTH_MEAN_RADIUS 6371.0072

#include <math.h>  // sqrt
#include <geographic_msgs/msg/geo_point.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <string>
#include "rclcpp/rclcpp.hpp"

namespace global_planner {

// GLOBAL PLANNER

template <typename T>
double squared(T x) {
  return x * x;
}

// Returns a weighted average of start and end, where ratio is the weight of
// start
inline double interpolate(double start, double end, double ratio) { return start + (end - start) * ratio; }

// Returns Map[key] if it exists, default_val otherwise
template <typename Key, typename Value, typename Map>
Value getWithDefault(Map& m, const Key& key, const Value& default_val) {
  if (m.find(key) != m.end()) {
    return m[key];
  }
  return default_val;
}

template <typename P>
void setPointCoordinates(P& point, double x, double y, double z) {
  point.x = x;
  point.y = y;
  point.z = z;
}

template <typename T>
T norm(T x, T y, T z) {
  return sqrt(squared(x) + squared(y) + squared(z));
}

template <typename P>
double norm(const P& p) {
  return norm(p.x, p.y, p.z);
}

// Returns a point between p1 and p2, ratio should be between 0 and 1
// ratio=0 -> p1, ratio=1 -> p2
template <typename P>
P interpolate(const P& p1, const P& p2, double ratio) {
  P new_point;
  new_point.x = interpolate(p1.x, p2.x, ratio);
  new_point.y = interpolate(p1.y, p2.y, ratio);
  new_point.z = interpolate(p1.z, p2.z, ratio);
  return new_point;
}

// Returns the point in the middle of the line segment between p1 and p2
template <typename P>
P middlePoint(const P& p1, const P& p2) {
  return interpolate(p1, p2, 0.5);
}

template <typename P1, typename P2>
P1 addPoints(const P1& p1, const P2& p2) {
  P1 new_p;
  new_p.x = p1.x + p2.x;
  new_p.y = p1.y + p2.y;
  new_p.z = p1.z + p2.z;
  return new_p;
}

template <typename P1, typename P2>
P1 subtractPoints(const P1& p1, const P2& p2) {
  P1 new_p;
  new_p.x = p1.x - p2.x;
  new_p.y = p1.y - p2.y;
  new_p.z = p1.z - p2.z;
  return new_p;
}

template <typename P, typename Float>
P scalePoint(const P& point, Float scalar) {
  P new_p;
  new_p.x = scalar * point.x;
  new_p.y = scalar * point.y;
  new_p.z = scalar * point.z;
  return new_p;
}

template <typename P>
double distance(const P& p1, const P& p2) {
  return norm((p2.x - p1.x), (p2.y - p1.y), (p2.z - p1.z));
}

inline double clocksToMicroSec(std::clock_t start, std::clock_t end) {
  return (end - start) / (double)(CLOCKS_PER_SEC / 1000000);
}

// returns angle in the range [-pi, pi]
inline double angleToRange(double angle) {
  angle += M_PI;
  angle -= (2 * M_PI) * std::floor(angle / (2 * M_PI));
  angle -= M_PI;
  return angle;
}

inline double posterior(double p, double prior) {
  // p and prior are independent measurements of the same event
  double prob_obstacle = p * prior;
  double prob_free = (1 - p) * (1 - prior);
  return prob_obstacle / (prob_obstacle + prob_free + 0.0001);
}

inline double distanceTo(geographic_msgs::msg::GeoPoint pos_ref, geographic_msgs::msg::GeoPoint pos_to) {
  // Haversine formula
  double dlat = (pos_to.latitude - pos_ref.latitude) * M_PI / 180;
  double dlon = (pos_to.longitude - pos_ref.longitude) * M_PI / 180;
  double haversine_dlat = sin(dlat / 2.0);
  haversine_dlat *= haversine_dlat;
  double haversine_dlon = sin(dlon / 2.0);
  haversine_dlon *= haversine_dlon;
  double y = haversine_dlat + cos(pos_ref.latitude * M_PI / 180) * cos(pos_to.latitude * M_PI / 180) * haversine_dlon;
  double x = 2 * asin(sqrt(y));
  return (x * EARTH_MEAN_RADIUS * 1000);
}

inline geographic_msgs::msg::GeoPoint atDistanceAndAzimuth(geographic_msgs::msg::GeoPoint pos, double distance,
                                                           double azimuth) {
  double latRad = pos.latitude * M_PI / 180;
  double lonRad = pos.longitude * M_PI / 180;
  double cosLatRad = cos(latRad);
  double sinLatRad = sin(latRad);

  double azimuthRad = azimuth * M_PI / 180;

  double ratio = (distance / (EARTH_MEAN_RADIUS * 1000.0));
  double cosRatio = cos(ratio);
  double sinRatio = sin(ratio);

  double resultLatRad = asin(sinLatRad * cosRatio + cosLatRad * sinRatio * cos(azimuthRad));
  double resultLonRad =
      lonRad + atan2(sin(azimuthRad) * sinRatio * cosLatRad, cosRatio - sinLatRad * sin(resultLatRad));

  pos.latitude = resultLatRad * 180 / M_PI;
  pos.longitude = resultLonRad * 180 / M_PI;
  return pos;
}

// geometry_msgs::msg::Point start_pos_
inline geometry_msgs::msg::Point LLH2NED(geographic_msgs::msg::GeoPoint pos_ref,
                                         geographic_msgs::msg::GeoPoint pos_to) {
  // Calc x,y,z of pos with refPos
  geographic_msgs::msg::GeoPoint pos_calc_X, pos_calc_Y;

  pos_calc_X.latitude = pos_to.latitude;
  pos_calc_X.longitude = pos_ref.longitude;
  pos_calc_X.altitude = pos_ref.altitude;

  pos_calc_Y.latitude = pos_ref.latitude;
  pos_calc_Y.longitude = pos_to.longitude;
  pos_calc_Y.altitude = pos_ref.latitude;

  double NED_X = distanceTo(pos_ref, pos_calc_X);
  if (pos_to.latitude < pos_ref.latitude) NED_X = -NED_X;
  double NED_Y = distanceTo(pos_ref, pos_calc_Y);
  if (pos_to.longitude < pos_ref.longitude) NED_Y = -NED_Y;
  double NED_Z = -(pos_to.altitude - pos_ref.altitude);

  geometry_msgs::msg::Point result_point;
  result_point.x = NED_X;
  result_point.y = NED_Y;
  result_point.z = NED_Z;
  return result_point;
}

inline geographic_msgs::msg::GeoPoint NED2LLH(geographic_msgs::msg::GeoPoint pos_ref,
                                              geometry_msgs::msg::Point pos_to) {
  geographic_msgs::msg::GeoPoint result_point;
  // Calc lat, lon, alt of pos with refPos

  // QGeoCoordinate LLHPosition = QGeoCoordinate(refPos.latitude(), refPos.longitude(), refPos.altitude());
  result_point = atDistanceAndAzimuth(pos_ref, pos_to.x, 0);
  result_point = atDistanceAndAzimuth(result_point, pos_to.y, 90);
  result_point.altitude = result_point.altitude - pos_to.z;

  return result_point;
}

}  // namespace global_planner

#endif /* GLOBAL_PLANNER_COMMON_H_ */