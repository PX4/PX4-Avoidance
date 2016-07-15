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

Node Node::nextNode(const Cell & nextCell) const {
  return Node(nextCell, cell_);
}

std::vector<Node> Node::getNeighbors() const {
  std::vector<Node> neighbors;
  neighbors.reserve(10);
	for (Cell neighborCell : cell_.getNeighbors()) {
		neighbors.push_back(nextNode(neighborCell));
	}
  return neighbors;
}

// The number of 45 degree turns neede to go to other
// Assumes that there is not both horizontal and vertical movement is needed
double Node::getRotation(const Node & other) const {
  double this_z_diff = cell_.z() - parent_.z();
  double other_z_diff = other.cell_.z() - other.parent_.z();
  if (this_z_diff != other_z_diff) {
    // TODO: use vert_to_hor_cost_
    return 1.0; // Change between horizontal and vertical movement
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
