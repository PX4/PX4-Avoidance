#ifndef GLOBAL_PLANNER_NODE
#define GLOBAL_PLANNER_NODE

#include <string>
#include <unordered_set>

#include "global_planner/cell.h"
#include "global_planner/common.h"

namespace global_planner {

class Node {
 public:
  Node() = default;
  Node(const Cell& cell, const Cell& parent) : cell_(cell), parent_(parent) {}
  virtual ~Node() = default;

  virtual bool isEqual(const Node& other) const;
  virtual bool isSmaller(const Node& other) const;
  virtual std::size_t hash() const;
  virtual std::shared_ptr<Node> nextNode(const Cell& nextCell) const;
  virtual std::vector<std::shared_ptr<Node> > getNeighbors() const;
  virtual std::unordered_set<Cell> getCells() const;

  virtual double getLength() const;
  virtual double getRotation(const Node& other) const;
  virtual double getXYRotation(const Node& other) const;
  std::string asString() const;

  Cell cell_;
  Cell parent_;
};

inline bool operator==(const Node& lhs, const Node& rhs) { return lhs.isEqual(rhs); }
inline bool operator<(const Node& lhs, const Node& rhs) { return lhs.isSmaller(rhs); }
inline bool operator!=(const Node& lhs, const Node& rhs) { return !operator==(lhs, rhs); }
inline bool operator>(const Node& lhs, const Node& rhs) { return operator<(rhs, lhs); }
inline bool operator<=(const Node& lhs, const Node& rhs) { return !operator>(lhs, rhs); }
inline bool operator>=(const Node& lhs, const Node& rhs) { return !operator<(lhs, rhs); }

typedef std::shared_ptr<Node> NodePtr;
typedef std::pair<Node, double> NodeDistancePair;
typedef std::pair<NodePtr, double> PointerNodeDistancePair;

class CompareDist {
 public:
  bool operator()(const CellDistancePair& n1, const CellDistancePair& n2) { return n1.second > n2.second; }
  bool operator()(const NodeDistancePair& n1, const NodeDistancePair& n2) { return n1.second > n2.second; }
  bool operator()(const PointerNodeDistancePair& n1, const PointerNodeDistancePair& n2) {
    return n1.second > n2.second;
  }
};

// Node that only represents 3D position, ignores parent
class NodeWithoutSmooth : public Node {
 public:
  NodeWithoutSmooth() = default;
  NodeWithoutSmooth(const Cell& cell, const Cell& parent) : Node(cell, parent) {}
  ~NodeWithoutSmooth() = default;

  bool isEqual(const Node& other) const { return cell_ == other.cell_; }

  std::size_t hash() const { return std::hash<global_planner::Cell>()(cell_); }

  NodePtr nextNode(const Cell& nextCell) const { return NodePtr(new NodeWithoutSmooth(nextCell, cell_)); }

  double getRotation(const Node& other) const { return 0.0; }
};

static double SPEEDNODE_RADIUS = 5.0;
// Node represents 3D position, orientation and speed
// TODO: Needs to check the risk of Cells between cell and parent
class SpeedNode : public Node {
 public:
  SpeedNode() = default;
  SpeedNode(const Cell& cell, const Cell& parent) : Node(cell, parent) {}
  ~SpeedNode() = default;

  NodePtr nextNode(const Cell& nextCell) const { return NodePtr(new SpeedNode(nextCell, cell_)); }

  std::vector<NodePtr> getNeighbors() const {
    std::vector<NodePtr> neighbors;
    Cell extrapolate_cell = (cell_ - parent_) + cell_;
    neighbors.push_back(nextNode(extrapolate_cell));
    for (const Cell& neighborCell : extrapolate_cell.getNeighbors()) {
      double dist = cell_.distance3D(neighborCell);
      if (dist > 0 && dist < SPEEDNODE_RADIUS) {
        neighbors.push_back(nextNode(neighborCell));
      }
    }
    return neighbors;
  }
};

struct HashNodePtr {
  std::size_t operator()(const std::shared_ptr<Node>& node_ptr) const { return node_ptr->hash(); }
};

struct EqualsNodePtr {
  std::size_t operator()(const std::shared_ptr<Node>& node_ptr1, const std::shared_ptr<Node>& node_ptr2) const {
    return node_ptr1->isEqual(*node_ptr2);
  }
};

}  // namespace global_planner

namespace std {

template <>
struct hash<global_planner::Node> {
  std::size_t operator()(const global_planner::Node& node) const { return node.hash(); }
};

template <>
struct hash<global_planner::NodeWithoutSmooth> {
  std::size_t operator()(const global_planner::NodeWithoutSmooth& node) const { return node.hash(); }
};

}  // namespace std

#endif  // GLOBAL_PLANNER_NODE
