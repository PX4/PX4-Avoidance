#include "local_planner/local_planner_node.h"

using namespace avoidance;

int main(int argc, char** argv) {
  ros::init(argc, argv, "local_planner_node");
  ros::NodeHandle nh("~");
  ros::NodeHandle nh_private("");

  LocalPlannerNode Node(nh, nh_private, true);
  Node.startNode();

  std::thread worker(&LocalPlannerNode::threadFunction, &Node);
  std::thread worker_tf_buffer(&LocalPlannerNode::transformBufferThread, &Node);

  worker.join();
  worker_tf_buffer.join();

  for (size_t i = 0; i < Node.cameras_.size(); ++i) {
    Node.cameras_[i].cloud_ready_cv_->notify_all();
    Node.cameras_[i].transform_thread_.join();
  }

  return 0;
}
