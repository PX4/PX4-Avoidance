#ifndef GLOBAL_PLANNER_VISITOR
#define GLOBAL_PLANNER_VISITOR

namespace global_planner {

template <typename Set, typename Map>
class SearchVisitor {
 public:
  Set seen_;
  Map seen_count_;

  SearchVisitor() {}
  void initVisitor(Set seen, Map seen_count) {}
  void init() {
    seen_.clear();
    seen_count_.clear();
  }
  void popNode(NodePtr u) {}
  void perNeighbor(NodePtr u, NodePtr v) {
    seen_count_[v->cell_] = 1.0 + getWithDefault(seen_count_, v->cell_, 0.0);
    seen_.insert(v->cell_);
  }
};

class NullVisitor {
 public:
  NullVisitor() {}

  void init() {}

  template <typename NodePtr>
  void popNode(NodePtr u) {}

  template <typename NodePtr>
  void perNeighbor(NodePtr u, NodePtr v) {}
};

}  // namespace global_planner

#endif  // GLOBAL_PLANNER_VISITOR
