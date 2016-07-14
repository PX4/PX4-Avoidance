#ifndef GLOBAL_PLANNER_CELL
#define GLOBAL_PLANNER_CELL

#include <math.h>           // abs
#include <string>
#include <tuple>

#include <geometry_msgs/Point.h>

#include "avoidance/common.h"

namespace avoidance {

class Cell {
 public:
  Cell();
  Cell(std::tuple<int, int, int> new_tuple);
  Cell(double x, double y, double z);
  Cell(double x, double y);
  Cell(geometry_msgs::Point point);
  // Cell(Eigen::Vector3d point);

  // Get the indices of the Cell
  int x() const;
  int y() const;
  int z() const;

  // Get the coordinates of the center-point of the Cell 
  double xPos() const;
  double yPos() const;
  double zPos() const;

  geometry_msgs::Point toPoint() const;

  double manhattanDist(double _x, double _y, double _z) const;
  double distance2D(const Cell & b) const;
  double distance3D(const Cell & b) const;
  double diagDistance2D(const Cell & b) const;
  double diagDistance3D(const Cell & b) const;
  double angle() const;
  
  Cell getNeighborFromYaw(double yaw) const;
  std::vector<Cell> getFlowNeighbors() const;
  std::vector<Cell> getDiagonalNeighbors() const;
  std::vector<Cell> getNeighbors() const;

  std::string asString() const;
  
  // Member variables
  std::tuple<int, int, int> tpl_;
  static constexpr double scale_ = 1.0;
};

inline bool operator==(const Cell & lhs, const Cell & rhs) {return lhs.tpl_ == rhs.tpl_;}
inline bool operator!=(const Cell & lhs, const Cell & rhs) {return !operator==(lhs,rhs);}
inline bool operator< (const Cell & lhs, const Cell & rhs) {return lhs.tpl_ < rhs.tpl_;}
inline bool operator> (const Cell & lhs, const Cell & rhs) {return  operator< (rhs,lhs);}
inline bool operator<=(const Cell & lhs, const Cell & rhs) {return !operator> (lhs,rhs);}
inline bool operator>=(const Cell & lhs, const Cell & rhs) {return !operator< (lhs,rhs);}

inline Cell operator+(const Cell& lhs, const Cell& rhs) {
  Cell res(std::tuple<int, int, int>(lhs.x() + rhs.x(), lhs.y() + rhs.y(), lhs.z() + rhs.z()));
  return res;
}
inline Cell operator-(const Cell& lhs, const Cell& rhs) {
  Cell res(std::tuple<int, int, int>(lhs.x() - rhs.x(), lhs.y() - rhs.y(), lhs.z() - rhs.z()));
  return res;
}

typedef std::pair<Cell, double> CellDistancePair;


} // namespace avoidance

namespace std {

template <>
struct hash<avoidance::Cell> {
    std::size_t operator()(const avoidance::Cell & cell ) const {
        return (std::get<0>(cell.tpl_) << 20) ^ (std::get<1>(cell.tpl_) << 10) ^ std::get<2>(cell.tpl_);
      // return (std::get<0>(cell.tpl_) * 18397) + (std::get<1>(cell.tpl_) * 20483) + (std::get<2>(cell.tpl_) * 29303);
    }
};

} // namespace std

#endif // GLOBAL_PLANNER_CELL
