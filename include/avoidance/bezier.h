#ifndef GLOBAL_PLANNER_BEZIER_H_
#define GLOBAL_PLANNER_BEZIER_H_

#include <math.h>   // sqrt

// This file consists functions for functions for Bezier curves

namespace avoidance {

// Returns the point on the quadratic Bezier curve at time t (0 <= t <= 1)
template <typename T>
T quadraticBezier(T p0, T p1, T p2, double t) {
  return ((1 - t) * (1 - t) * p0) + 2 * ((1 - t) * t * p1) + (t * t * p2);
}

// Returns the point on the quadratic Bezier curve, actually independant of t
template <typename T>
T quadraticBezierAcc(T p0, T p1, T p2, double duration = 1.0) {
  return 2 * (p2 - 2*p1 + p0) / duration*duration;
}

// Returns a quadratic Bezier-curve starting in p0 and and ending in p2
template <typename P>
std::vector<P> threePointBezier(const P & p0, const P & p1, const P & p2, int num_steps = 10) {
  std::vector<P> curve;
  for (int i=0; i <= num_steps; ++i) {
    double t = ((double) i) / num_steps;
    P new_point;
    new_point.x = quadraticBezier(p0.x, p1.x, p2.x, t);
    new_point.y = quadraticBezier(p0.y, p1.y, p2.y, t);
    new_point.z = quadraticBezier(p0.z, p1.z, p2.z, t);
    curve.push_back(new_point);
  }
  return curve;
}

// Returns a quadratic Bezier-curve starting in p0 and and ending in p2
template <typename Path>
nav_msgs::Path threePointBezier(const Path & path, int num_steps = 10) {
  if (path.poses.size() != 3) {
    printf("Path size error, %d != 3 \n", path.poses.size());
    return path;
  }
  auto new_path = path;
  auto new_points = threePointBezier(new_path.poses[0].pose.position,
                                     new_path.poses[1].pose.position,
                                     new_path.poses[2].pose.position,
                                     num_steps); 
  new_path.poses.clear();
  for (auto point : new_points) {
    auto new_pose = path.poses[0];
    new_pose.pose.position = point;
    new_path.poses.push_back(new_pose);
  }
  return new_path;
}

template <typename P, typename BezierMsg>
void fillBezierMsg(BezierMsg & msg, const P & p0, const P & p1, const P & p2, double duration) {
  msg.prev = p0;
  msg.ctrl = p1;
  msg.next = p2;
  msg.duration = duration;
}

// Fills msgs with three BezierMsgs, 
// The first message represent accelerating to max_vel
// The second is maintaining max_vel
// The third is decelerating and halting at end
template <typename P, typename BezierMsg>
void bezierFromTwoPoints(const P & start, const P & end,
                      double acc, double max_vel, std::vector<BezierMsg> & msgs) {

  double total_dist = distance(start, end);
  
  // The duration and distance needed to accerlerate to max_vel
  double acc_duration = max_vel / acc;
  double acc_dist = acc_duration * max_vel / 2;
  // The acceleration phase cannot be more than 50% of the trajectory
  double acc_part = std::min(0.5, acc_dist / total_dist);
  
  // We accelerate from start to max_vel_point, keep max_vel till decel_point, decelerate to end
  P middle = middlePoint(start, end);
  P max_vel_point = interpolate(start, end, acc_part);
  P decel_point = interpolate(end, start, acc_part);
  double max_vel_duration = distance(max_vel_point, decel_point) / max_vel;

  // Fill the messages  
  BezierMsg acc_msg, max_vel_msg, decel_msg;
  fillBezierMsg(acc_msg, start, start, max_vel_point, acc_duration);
  fillBezierMsg(max_vel_msg, max_vel_point, middle, decel_point, max_vel_duration);
  fillBezierMsg(decel_msg, decel_point, end, end, acc_duration);
  msgs = {acc_msg, max_vel_msg, decel_msg};
}

// The time it takes to accelerate from p0 to p1, starting with no velocity at p0
template <typename P>
double getDuration(const P & p0, const P & p1, double acc) {
  double dist = distance(p0, p1);
  return sqrt(2 * dist / acc);
}

template <typename P>
double getAcceleration(const P & p0, const P & p1, const P & p2) {
  double dx = quadraticBezierAcc(p0.x, p1.x, p2.x);
  double dy = quadraticBezierAcc(p0.y, p1.y, p2.y);
  double dz = quadraticBezierAcc(p0.z, p1.z, p2.z);
  return sqrt(dx*dx + dy*dy + dz*dz);
}

} // namespace avoidance
#endif /* GLOBAL_PLANNER_BEZIER_H_ */
