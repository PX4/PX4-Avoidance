#ifndef TREE_NODE_H
#define TREE_NODE_H

#include <vector>
#include <math.h>
#include <geometry_msgs/Point.h>

class TreeNode
{

  geometry_msgs::Point position;

 public:
  double total_cost;
  double heuristic;
  int last_e;
  int last_z;
  int origin;
  int depth;
  double yaw;

  TreeNode();
  TreeNode(int from, int d, geometry_msgs::Point pos);
  ~TreeNode();

 void setCosts(double h, double c);
 geometry_msgs::Point getPosition();
};


#endif // TREE_NODE_H
