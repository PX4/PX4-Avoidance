#include "mock_data_node.h"

namespace global_planner {

MockDataNode::MockDataNode() {
  ros::NodeHandle nh;

  path_sub_ = nh.subscribe("global_path", 1, &MockDataNode::receivePath, this);

  depth_points_pub_ = nh.advertise<sensor_msgs::PointCloud2>("camera/depth/points", 10);
  local_position_pub_ = nh.advertise<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10);
  global_goal_pub_ = nh.advertise<geometry_msgs::PointStamped>("clicked_point", 10);

  createWall(5, 5, 6);

  int num_loops = 0;
  ros::Rate rate(1);
  while (ros::ok()) {
    if (num_loops++ == 10) {
      sendClickedPoint();
    }
    sendMockData();
    rate.sleep();
    ros::spinOnce();
  }
}

MockDataNode::~MockDataNode() {}

void MockDataNode::createWall(int dist, int width, int height) {
  points_.clear();
  for (int i = -width; i <= width; ++i) {
    for (int j = 0; j <= height; ++j) {
      points_.push_back(dist + 0.5);
      points_.push_back(i + 0.5);
      points_.push_back(j + 0.5);
    }
  }
}

void MockDataNode::sendClickedPoint() {
  geometry_msgs::PointStamped msg;
  msg.header.frame_id = "world";
  msg.point.x = 8.5;
  msg.point.y = 4.5;
  msg.point.z = 1.5;
  global_goal_pub_.publish(msg);
}

void MockDataNode::receivePath(const nav_msgs::Path& msg) {
  for (auto p : msg.poses) {
    double x = p.pose.position.x;
    double y = p.pose.position.y;
    double z = p.pose.position.z;
    printf("(%2.2f, %2.2f, %2.2f) -> ", x, y, z);
  }
  printf("\n\n");
}

void MockDataNode::sendMockData() {
  // Create a PointCloud2
  sensor_msgs::PointCloud2 cloud_msg;
  cloud_msg.header.frame_id = "world";
  // Fill some internals of the PoinCloud2 like the header/width/height ...
  cloud_msg.height = 1;
  cloud_msg.width = 4;
  // Set the point fields to xyzrgb and resize the vector with the following
  // command 4 is for the number of added fields. Each come in triplet: the name
  // of the PointField, the number of occurences of the type in the PointField,
  // the type of the PointField
  sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
  modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1, sensor_msgs::PointField::FLOAT32,
                                "z", 1, sensor_msgs::PointField::FLOAT32, "rgb", 1, sensor_msgs::PointField::FLOAT32);
  // For convenience and the xyz, rgb, rgba fields, you can also use the
  // following overloaded function. You have to be aware that the following
  // function does add extra padding for backward compatibility though so it is
  // definitely the solution of choice for PointXYZ and PointXYZRGB 2 is for the
  // number of fields to add
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  // You can then reserve / resize as usual
  modifier.resize(100);

  // Define some raw data we'll put in the PointCloud2
  int n = points_.size() / 3;
  uint8_t color_data[] = {40, 200, 120};

  // Define the iterators. When doing so, you define the Field you would like to
  // iterate upon and the type of you would like returned: it is not necessary
  // the type of the PointField as sometimes you pack data in another type (e.g.
  // 3 uchar + 1 uchar for RGB are packed in a float)
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
  // Even though the r,g,b,a fields do not exist (it's usually rgb, rgba), you
  // can create iterators for those: they will handle data packing for you (in
  // little endian RGB is packed as *,R,G,B in a float and RGBA as A,R,G,B)
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_msg, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_msg, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_msg, "b");
  // Fill the PointCloud2
  for (size_t i = 0; i < n; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b) {
    *iter_x = points_[3 * i + 0];
    *iter_y = points_[3 * i + 1];
    *iter_z = points_[3 * i + 2];
    *iter_r = color_data[0];
    *iter_g = color_data[1];
    *iter_b = color_data[2];
  }

  depth_points_pub_.publish(cloud_msg);

  // Send position
  geometry_msgs::PoseStamped pos;
  pos.header.frame_id = "world";
  pos.pose.position.x = 0.5;
  pos.pose.position.y = 2.5;
  pos.pose.position.z = 1.5;
  pos.pose.orientation.w = 1.0;
  local_position_pub_.publish(pos);
}

}  // namespace global_planner

int main(int argc, char** argv) {
  ros::init(argc, argv, "mock_data_node");
  global_planner::MockDataNode mock_data_node;
  ros::spin();
  return 0;
}
