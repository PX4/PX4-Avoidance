#ifndef GLOBAL_PLANNER_COMMON_H_
#define GLOBAL_PLANNER_COMMON_H_

#include <math.h>  // sqrt
#include <string>

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

}  // namespace global_planner

#endif /* GLOBAL_PLANNER_COMMON_H_ */
