#include "global_planner/cell.h"

namespace global_planner {

Cell::Cell() = default;
Cell::Cell(std::tuple<int, int, int> new_tuple) : tpl_(new_tuple) {}
Cell::Cell(double x, double y, double z) : tpl_(floor(x / CELL_SCALE), floor(y / CELL_SCALE), floor(z / CELL_SCALE)) {}
Cell::Cell(double x, double y) : Cell(x, y, 0.0) {}
Cell::Cell(geometry_msgs::Point point) : Cell(point.x, point.y, point.z) {}

int Cell::xIndex() const { return std::get<0>(tpl_); }
int Cell::yIndex() const { return std::get<1>(tpl_); }
int Cell::zIndex() const { return std::get<2>(tpl_); }

double Cell::xPos() const { return CELL_SCALE * (xIndex() + 0.5); }
double Cell::yPos() const { return CELL_SCALE * (yIndex() + 0.5); }
double Cell::zPos() const { return CELL_SCALE * (zIndex() + 0.5); }

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

// Returns the straight-line distance, disregarding the z-coordinate, to the
// center of the Cell
double Cell::distance2D(const Cell& b) const { return sqrt(squared(xPos() - b.xPos()) + squared(yPos() - b.yPos())); }

double Cell::distance3D(const Cell& b) const {
  return sqrt(squared(xPos() - b.xPos()) + squared(yPos() - b.yPos()) + squared(zPos() - b.zPos()));
}

// Returns the minimum distance on the XY-grid, where you can move diagonally,
// to the center of the Cell
double Cell::diagDistance2D(const Cell& b) const {
  double dx = abs(xPos() - b.xPos());
  double dy = abs(yPos() - b.yPos());
  double diag_cost = 1.41421356237;
  return (dx + dy) + (diag_cost - 2) * std::min(dx, dy);
}

// Returns the minimum distance on the XY-grid, where you can move diagonally,
// to the center of the Cell
double Cell::diagDistance3D(const Cell& b) const { return diagDistance2D(b) + abs(zPos() - b.zPos()); }

// Returns the angle in the XY-plane between the Cell and the X-axis
// Cell at position (0,1,1) has the angle PI / 2
double Cell::angle() const { return atan2(yIndex(), xIndex()); }

// Returns the neighboring cell in the yaw direction
// E.g. if yaw == PI/4, then it returns Cell(x+1, y+1, z)
Cell Cell::getNeighborFromYaw(double yaw) const {
  int dx = 2 * CELL_SCALE * std::cos(yaw);
  int dy = 2 * CELL_SCALE * std::sin(yaw);
  return Cell(xPos() + dx, yPos() + dy, zPos());
}

// Returns the neighbors of the Cell whose risk influences the Cell
std::vector<Cell> Cell::getFlowNeighbors(int radius) const {
  std::vector<Cell> cells;
  auto ceilDistance = [](int radius, int x, int y) {
    auto sqr = [](int i) { return double(i * i); };
    return static_cast<int>(std::ceil(std::sqrt(sqr(radius) - sqr(x) - sqr(y))));
  };
  for (int x = -radius; x <= radius; x++) {
    int y_radius = ceilDistance(radius, x, 0);
    for (int y = -y_radius; y <= y_radius; y++) {
      int z_radius = ceilDistance(radius, x, y);
      for (int z = -z_radius; z <= z_radius; z++) {
        cells.push_back(Cell(std::tuple<int, int, int>(xIndex() + x, yIndex() + y, zIndex() + z)));
      }
    }
  }
  return cells;
}

// Returns the neighbors of the Cell that are diagonal to the cell in the
// XY-plane
std::vector<Cell> Cell::getDiagonalNeighbors() const {
  return std::vector<Cell>{Cell(std::tuple<int, int, int>(xIndex() + 1, yIndex() + 1, zIndex())),
                           Cell(std::tuple<int, int, int>(xIndex() - 1, yIndex() + 1, zIndex())),
                           Cell(std::tuple<int, int, int>(xIndex() + 1, yIndex() - 1, zIndex())),
                           Cell(std::tuple<int, int, int>(xIndex() - 1, yIndex() - 1, zIndex()))};
}

std::vector<Cell> Cell::getNeighbors() const {
  return std::vector<Cell>{Cell(std::tuple<int, int, int>(xIndex() + 1, yIndex(), zIndex())),
                           Cell(std::tuple<int, int, int>(xIndex() - 1, yIndex(), zIndex())),
                           Cell(std::tuple<int, int, int>(xIndex(), yIndex() + 1, zIndex())),
                           Cell(std::tuple<int, int, int>(xIndex(), yIndex() - 1, zIndex())),
                           Cell(std::tuple<int, int, int>(xIndex(), yIndex(), zIndex() + 1)),
                           Cell(std::tuple<int, int, int>(xIndex(), yIndex(), zIndex() - 1)),
                           Cell(std::tuple<int, int, int>(xIndex() + 1, yIndex() + 1, zIndex())),
                           Cell(std::tuple<int, int, int>(xIndex() - 1, yIndex() + 1, zIndex())),
                           Cell(std::tuple<int, int, int>(xIndex() + 1, yIndex() - 1, zIndex())),
                           Cell(std::tuple<int, int, int>(xIndex() - 1, yIndex() - 1, zIndex()))};
}

std::string Cell::asString() const {
  std::string s =
      "(" + std::to_string(xIndex()) + "," + std::to_string(yIndex()) + "," + std::to_string(zIndex()) + ")";
  return s;
}

}  // namespace global_planner
