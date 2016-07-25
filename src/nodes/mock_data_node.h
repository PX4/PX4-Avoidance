#ifndef GLOBAL_PLANNER_MOCK_DATA_NODE_H
#define GLOBAL_PLANNER_MOCK_DATA_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <nav_msgs/Path.h>

#include <vector>
#include <stdlib.h> 

#include "avoidance/common.h" // hasSameYawAndAltitude

namespace avoidance {

class MockDataNode {
 public:
  MockDataNode();
  ~MockDataNode();
  void createWall(int dist, int width, int height);
  void sendClickedPoint();
  void ReceivePath(const nav_msgs::Path & msg);
  void sendMockData();
  

  std::vector<float> points_ {5.5,-0.5,0.5, 5.5,0.5,0.5, 5.5,1.5,0.5,  
                             5.5,-0.5,1.5, 5.5,0.5,1.5, 5.5,1.5,1.5,
                             5.5,-0.5,2.5, 5.5,0.5,2.5, 5.5,1.5,2.5};

 private:
  ros::Subscriber path_sub_;

  ros::Publisher local_position_pub_;
  ros::Publisher depth_points_pub_;
  ros::Publisher global_goal_pub_;
};

} // namespace avoidance

#endif // GLOBAL_PLANNER_MOCK_DATA_NODE_H
