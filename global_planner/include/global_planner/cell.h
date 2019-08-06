#ifndef GLOBAL_PLANNER_CELL
#define GLOBAL_PLANNER_CELL

#include <math.h>  // abs
#include <string>
#include <tuple>

#include <geometry_msgs/Point.h>

#include "global_planner/common.h"

namespace global_planner {

static double CELL_SCALE = 1.0;

class Cell {
 public:
  Cell();
  Cell(std::tuple<int, int, int> new_tuple);
  Cell(double x, double y, double z);
  Cell(double x, double y);
  Cell(geometry_msgs::Point point);
  // Cell(Eigen::Vector3d point);

  // Get the indices of the Cell
  int xIndex() const;
  int yIndex() const;
  int zIndex() const;

  // Get the coordinates of the center-point of the Cell
  double xPos() const;
  double yPos() const;
  double zPos() const;

  geometry_msgs::Point toPoint() const;

  double manhattanDist(double _x, double _y, double _z) const;
  double distance2D(const Cell& b) const;
  double distance3D(const Cell& b) const;
  double diagDistance2D(const Cell& b) const;
  double diagDistance3D(const Cell& b) const;
  double angle() const;

  Cell getNeighborFromYaw(double yaw) const;
  std::vector<Cell> getFlowNeighbors(int radius) const;
  std::vector<Cell> getDiagonalNeighbors() const;
  std::vector<Cell> getNeighbors() const;

  std::string asString() const;

  // Member variables
  std::tuple<int, int, int> tpl_;
};

inline bool operator==(const Cell& lhs, const Cell& rhs) { return lhs.tpl_ == rhs.tpl_; }
inline bool operator!=(const Cell& lhs, const Cell& rhs) { return !operator==(lhs, rhs); }
inline bool operator<(const Cell& lhs, const Cell& rhs) { return lhs.tpl_ < rhs.tpl_; }
inline bool operator>(const Cell& lhs, const Cell& rhs) { return operator<(rhs, lhs); }
inline bool operator<=(const Cell& lhs, const Cell& rhs) { return !operator>(lhs, rhs); }
inline bool operator>=(const Cell& lhs, const Cell& rhs) { return !operator<(lhs, rhs); }

inline Cell operator+(const Cell& lhs, const Cell& rhs) {
  Cell res(
      std::tuple<int, int, int>(lhs.xIndex() + rhs.xIndex(), lhs.yIndex() + rhs.yIndex(), lhs.zIndex() + rhs.zIndex()));
  return res;
}
inline Cell operator-(const Cell& lhs, const Cell& rhs) {
  Cell res(
      std::tuple<int, int, int>(lhs.xIndex() - rhs.xIndex(), lhs.yIndex() - rhs.yIndex(), lhs.zIndex() - rhs.zIndex()));
  return res;
}

typedef std::pair<Cell, double> CellDistancePair;

// A GoalCell has a radius and can check if a position or another Cell is inside
// its radius
class GoalCell : public Cell {
 public:
  GoalCell(Cell cell, double radius = 1.0, bool is_temporary = false)
      : Cell(cell), radius_(radius), is_temporary_(is_temporary) {}

  GoalCell(double x, double y, double z, double radius = 1.0, bool is_temporary = false)
      : Cell(x, y, z), radius_(radius), is_temporary_(is_temporary) {}

  bool withinPlanRadius(Cell cell) const {
    return manhattanDist(cell.xPos(), cell.yPos(), cell.zPos()) < radius_ / 2.0;
  }

  template <typename P>
  bool withinPositionRadius(P point) const {
    return manhattanDist(point.x, point.y, point.z) < radius_;
  }

  double radius_;
  bool is_temporary_;
};

}  // namespace global_planner

namespace std {

template <>
struct hash<global_planner::Cell> {
  std::size_t operator()(const global_planner::Cell& cell) const {
    return (std::get<0>(cell.tpl_) << 20) ^ (std::get<1>(cell.tpl_) << 10) ^ std::get<2>(cell.tpl_);
    // return (std::get<0>(cell.tpl_) * 18397) + (std::get<1>(cell.tpl_) *
    // 20483) + (std::get<2>(cell.tpl_) * 29303);
  }
};

}  // namespace std

#endif  // GLOBAL_PLANNER_CELL
