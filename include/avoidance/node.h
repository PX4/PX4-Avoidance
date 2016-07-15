#ifndef GLOBAL_PLANNER_NODE
#define GLOBAL_PLANNER_NODE

#include <string>

#include "avoidance/cell.h"
#include "avoidance/common.h"

namespace avoidance {
  
class Node {
 public:
  Node() = default;
  Node(const Cell & cell, const Cell & parent) : cell_(cell), parent_(parent) {}

  std::vector<Node> getNeighbors() const;
  double getRotation(const Node & other) const;
  double getXYRotation(const Node & other) const;
  std::string asString() const;

  Cell cell_;
  Cell parent_;
};

inline bool operator==(const Node & lhs, const Node & rhs) {
  return lhs.cell_ == rhs.cell_ && lhs.parent_ == rhs.parent_;}
inline bool operator< (const Node & lhs, const Node & rhs) {
  return lhs.cell_ < rhs.cell_ || (lhs.cell_ == rhs.cell_ && lhs.parent_ < rhs.parent_);}
inline bool operator!=(const Node & lhs, const Node & rhs) {return !operator==(lhs, rhs);}
inline bool operator> (const Node & lhs, const Node & rhs) {return  operator< (rhs, lhs);}
inline bool operator<=(const Node & lhs, const Node & rhs) {return !operator> (lhs, rhs);}
inline bool operator>=(const Node & lhs, const Node & rhs) {return !operator< (lhs, rhs);}

typedef std::pair<Node, double> NodeDistancePair;

class CompareDist {
 public:
  bool operator()(const CellDistancePair & n1, const CellDistancePair & n2) {
    return n1.second > n2.second;
  }
  bool operator()(const NodeDistancePair & n1, const NodeDistancePair & n2) {
    return n1.second > n2.second;
  }
};

} // namespace avoidance

namespace std {

template <>
struct hash<avoidance::Node> {
    std::size_t operator()(const avoidance::Node & node ) const {
        return (std::hash<avoidance::Cell>()(node.cell_) << 1) ^ std::hash<avoidance::Cell>()(node.parent_);
        // return std::hash<avoidance::Cell>()(node.cell_) * 37 + std::hash<avoidance::Cell>()(node.parent_) * 41;
    }
};

} // namespace std

#endif // GLOBAL_PLANNER_NODE
