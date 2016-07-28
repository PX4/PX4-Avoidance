#include "avoidance/node.h"

namespace avoidance {

bool Node::isSmaller(const Node & other) const {
  return cell_ < other.cell_ || (cell_ == other.cell_ && parent_ < other.parent_);
}
bool Node::isEqual(const Node & other) const {
  return cell_ == other.cell_ && parent_ == other.parent_;
}

std::size_t Node::hash() const {
  return (std::hash<avoidance::Cell>()(cell_) << 1) ^ std::hash<avoidance::Cell>()(parent_);
  // return std::hash<avoidance::Cell>()(node.cell_) * 37 + std::hash<avoidance::Cell>()(node.parent_) * 41;
}

NodePtr Node::nextNode(const Cell & nextCell) const {
  return NodePtr(new Node(nextCell, cell_));
}

std::vector<NodePtr> Node::getNeighbors() const {
  std::vector<NodePtr > neighbors;
  neighbors.reserve(10);
	for (Cell neighborCell : cell_.getNeighbors()) {
		neighbors.push_back(nextNode(neighborCell));
	}
  return neighbors;
}

std::unordered_set<Cell> Node::getCells() const {
  std::unordered_set<Cell> cells;
  int dx = cell_.x() - parent_.x();
  int dy = cell_.y() - parent_.y();
  int dz = cell_.z() - parent_.z();

  int steps = 2 * std::max(std::abs(dx), std::max(std::abs(dy), std::abs(dz)));
  
  double x_step = (cell_.xPos() - parent_.xPos()) / steps;
  double y_step = (cell_.yPos() - parent_.yPos()) / steps;
  double z_step = (cell_.zPos() - parent_.zPos()) / steps;

  for (int i = 1; i <= steps; ++i) {
    double new_x = parent_.xPos() + x_step * i;
    double new_y = parent_.yPos() + y_step * i;
    double new_z = parent_.zPos() + z_step * i;
    cells.insert(Cell(new_x + 0.1, new_y + 0.1, new_z));
    cells.insert(Cell(new_x + 0.1, new_y - 0.1, new_z));
    cells.insert(Cell(new_x - 0.1, new_y + 0.1, new_z));
    cells.insert(Cell(new_x - 0.1, new_y - 0.1, new_z));
  }
  return cells;
}

double Node::getLength() const {
  return parent_.distance3D(cell_);
}

// The number of 45 degree turns neede to go to other
// Assumes that there is not both horizontal and vertical movement is needed
double Node::getRotation(const Node & other) const {
  double this_z_diff = cell_.z() - parent_.z();
  double other_z_diff = other.cell_.z() - other.parent_.z();
  if (this_z_diff != other_z_diff) {
    // TODO: use vert_to_hor_cost_
    return 0.5; // Change between horizontal and vertical movement
  }
  return getXYRotation(other);
}

double Node::getXYRotation(const Node & other) const {
  double this_ang = (cell_ - parent_).angle();
  double other_ang = (other.cell_ - other.parent_).angle();
  double ang_diff = other_ang - this_ang;
  ang_diff = std::fabs(angleToRange(ang_diff));   // Rotation needed
  double num_45_deg_turns = ang_diff / (M_PI/4);  // Minimum number of 45-turns to goal
  return num_45_deg_turns;
}

std::string Node::asString() const {
  std::string s = "(" + cell_.asString() + " , " + parent_.asString() + ")";
  return s;
}

} // namespace avoidance
