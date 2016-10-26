#include "avoidance/cell.h"

namespace avoidance {

Cell::Cell() = default;
Cell::Cell(std::tuple<int, int, int> new_tuple)
  : tpl_(new_tuple) {}
Cell::Cell(double x, double y, double z)
  : tpl_(floor(x / CELL_SCALE), floor(y / CELL_SCALE), floor(z / CELL_SCALE)) {}
Cell::Cell(double x, double y) 
  : Cell(x, y, 0.0) {}
Cell::Cell(geometry_msgs::Point point) 
  : Cell(point.x, point.y, point.z) {}
// Cell::Cell(Eigen::Vector3d point) 
  // : Cell(point[0], point[1], point[2])  {}

int Cell::x() const {return std::get<0>(tpl_);}
int Cell::y() const {return std::get<1>(tpl_);}
int Cell::z() const {return std::get<2>(tpl_);}

double Cell::xPos() const {return CELL_SCALE * (x() + 0.5);}
double Cell::yPos() const {return CELL_SCALE * (y() + 0.5);}
double Cell::zPos() const {return CELL_SCALE * (z() + 0.5);}

geometry_msgs::Point Cell::toPoint() const {
  geometry_msgs::Point point;
  point.x = xPos();
  point.y = yPos();
  point.z = zPos();
  return point;
}

// Returns the Manhattan-distance from the center of the Cell
double Cell::manhattanDist(double _x, double _y, double _z) const {
  return std::abs(xPos() - _x) + std::abs(yPos() - _y) + std::abs(zPos() - _z);
}

// Returns the straight-line distance, disregarding the z-coordinate, to the center of the Cell
double Cell::distance2D(const Cell & b) const {
  return sqrt(squared(xPos() - b.xPos()) + squared(yPos() - b.yPos()));
}

double Cell::distance3D(const Cell & b) const {
  return sqrt(squared(xPos() - b.xPos()) + squared(yPos() - b.yPos()) + squared(zPos() - b.zPos()));
}

// Returns the minimum distance on the XY-grid, where you can move diagonally, to the center of the Cell
double Cell::diagDistance2D(const Cell & b) const {
  double dx = abs(xPos() - b.xPos());
  double dy = abs(yPos() - b.yPos());
  double diag_cost = 1.41421356237;
  return (dx + dy) + (diag_cost - 2) * std::min(dx, dy);
}

// Returns the minimum distance on the XY-grid, where you can move diagonally, to the center of the Cell
double Cell::diagDistance3D(const Cell & b) const {
  return diagDistance2D(b) + abs(zPos() - b.zPos());
}

// Returns the angle in the XY-plane between the Cell and the X-axis
// Cell at position (0,1,1) has the angle PI / 2  
double Cell::angle() const {
  return atan2(y(), x());
}

// Returns the neighboring cell in the yaw direction
// E.g. if yaw == PI/4, then it returns Cell(x+1, y+1, z)
Cell Cell::getNeighborFromYaw(double yaw) const {
  int dx = 2 * CELL_SCALE * std::cos(yaw);
  int dy = 2 * CELL_SCALE * std::sin(yaw);
  return Cell(xPos() + dx, yPos() + dy, zPos());
}

// Returns the neighbors of the Cell whose risk influences the Cell
std::vector<Cell> Cell::getFlowNeighbors() const {
  return std::vector<Cell>{Cell(std::tuple<int,int,int>(x() + 1, y(), z())),  
                           Cell(std::tuple<int,int,int>(x() - 1, y(), z())),  
                           Cell(std::tuple<int,int,int>(x(), y() + 1, z())),  
                           Cell(std::tuple<int,int,int>(x(), y() - 1, z())),  
                           Cell(std::tuple<int,int,int>(x(), y(), z() + 1)),  
                           Cell(std::tuple<int,int,int>(x(), y(), z() - 1))
                          };
}

// Returns the neighbors of the Cell that are diagonal to the cell in the XY-plane
std::vector<Cell> Cell::getDiagonalNeighbors() const {
  return std::vector<Cell>{Cell(std::tuple<int,int,int>(x() + 1, y() + 1, z())),
                           Cell(std::tuple<int,int,int>(x() - 1, y() + 1, z())),
                           Cell(std::tuple<int,int,int>(x() + 1, y() - 1, z())),
                           Cell(std::tuple<int,int,int>(x() - 1, y() - 1, z()))
                          };
}

std::vector<Cell> Cell::getNeighbors() const {
  return std::vector<Cell>{Cell(std::tuple<int,int,int>(x() + 1, y(), z())),
                           Cell(std::tuple<int,int,int>(x() - 1, y(), z())),
                           Cell(std::tuple<int,int,int>(x(), y() + 1, z())),
                           Cell(std::tuple<int,int,int>(x(), y() - 1, z())),
                           Cell(std::tuple<int,int,int>(x(), y(), z() + 1)),
                           Cell(std::tuple<int,int,int>(x(), y(), z() - 1)),
                           Cell(std::tuple<int,int,int>(x() + 1, y() + 1, z())),
                           Cell(std::tuple<int,int,int>(x() - 1, y() + 1, z())),
                           Cell(std::tuple<int,int,int>(x() + 1, y() - 1, z())),
                           Cell(std::tuple<int,int,int>(x() - 1, y() - 1, z()))
                          };
}

std::string Cell::asString() const {  
  std::string s = "(" + std::to_string(x()) + "," + std::to_string(y()) + "," + std::to_string(z()) + ")";
  return s;
}

} // namespace avoidance
