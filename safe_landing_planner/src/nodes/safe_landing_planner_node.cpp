#include "safe_landing_planner/safe_landing_planner_node.hpp"

namespace avoidance {

const Eigen::Vector3f nan_setpoint = Eigen::Vector3f(NAN, NAN, NAN);

SafeLandingPlannerNode::SafeLandingPlannerNode(const ros::NodeHandle &nh) : nh_(nh), spin_dt_(0.1) {
  safe_landing_planner_.reset(new SafeLandingPlanner());

#ifndef DISABLE_SIMULATION
  world_visualizer_.reset(new avoidance::WorldVisualizer(nh_, ros::this_node::getName()));
#endif
  visualizer_.initializePublishers(nh_);

  std::string camera_topic;
  nh_.getParam("pointcloud_topics", camera_topic);
  nh_.param<bool>("play_rosbag", safe_landing_planner_->play_rosbag_, false);

  dynamic_reconfigure::Server<safe_landing_planner::SafeLandingPlannerNodeConfig>::CallbackType f;
  f = boost::bind(&SafeLandingPlannerNode::dynamicReconfigureCallback, this, _1, _2);
  server_.setCallback(f);

  pose_sub_ = nh_.subscribe<const geometry_msgs::PoseStamped &>("mavros/local_position/pose", 1,
                                                                &SafeLandingPlannerNode::positionCallback, this);
  pointcloud_sub_ = nh_.subscribe<const sensor_msgs::PointCloud2 &>(camera_topic, 1,
                                                                    &SafeLandingPlannerNode::pointCloudCallback, this);

  mavros_system_status_pub_ = nh_.advertise<mavros_msgs::CompanionProcessStatus>("mavros/companion_process/status", 1);
  grid_pub_ = nh_.advertise<safe_landing_planner::SLPGridMsg>("grid_slp", 1);

  if (safe_landing_planner_->play_rosbag_) {
    raw_grid_sub_ = nh_.subscribe<const safe_landing_planner::SLPGridMsg &>(
        "/raw_grid_slp", 1, &SafeLandingPlannerNode::rawGridCallback, this);
    pointcloud_sub_.shutdown();
  }

  start_time_ = ros::Time::now();
}

void SafeLandingPlannerNode::dynamicReconfigureCallback(safe_landing_planner::SafeLandingPlannerNodeConfig &config,
                                                        uint32_t level) {
  rqt_param_config_ = config;
  safe_landing_planner_->dynamicReconfigureSetParams(config, level);
}

void SafeLandingPlannerNode::positionCallback(const geometry_msgs::PoseStamped &msg) {
  previous_pose_ = current_pose_;
  current_pose_ = msg;
  position_received_ = true;
}

void SafeLandingPlannerNode::pointCloudCallback(const sensor_msgs::PointCloud2 &msg) {
  {
    std::lock_guard<std::mutex> lck(*(cloud_msg_mutex_));
    newest_cloud_msg_ = msg;  // FIXME: avoid a copy
  }

  cloud_ready_cv_->notify_one();
}

void SafeLandingPlannerNode::rawGridCallback(const safe_landing_planner::SLPGridMsg &msg) {
  std::lock_guard<std::mutex> transformed_cloud_guard(*(transformed_cloud_mutex_));
  safe_landing_planner_->raw_grid_ = std::move(msg);
  cloud_transformed_ = true;
}

void SafeLandingPlannerNode::startNode() {
  // initialize thread
  worker_ = std::thread(&SafeLandingPlannerNode::pointCloudTransformThread, this);
  cloud_msg_mutex_.reset(new std::mutex);
  transformed_cloud_mutex_.reset(new std::mutex);
  cloud_ready_cv_.reset(new std::condition_variable);

  ros::TimerOptions timer_options(ros::Duration(spin_dt_),
                                  boost::bind(&SafeLandingPlannerNode::cmdLoopCallback, this, _1), &cmdloop_queue_);
  cmdloop_timer_ = nh_.createTimer(timer_options);
  cmdloop_spinner_.reset(new ros::AsyncSpinner(1, &cmdloop_queue_));
  cmdloop_spinner_->start();
}

void SafeLandingPlannerNode::cmdLoopCallback(const ros::TimerEvent &event) {
  status_msg_.state = static_cast<int>(avoidance::MAV_STATE::MAV_STATE_ACTIVE);

  ros::Time start_query_position = ros::Time::now();
  while (!cloud_transformed_ && ros::ok()) {
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
    ros::Duration since_query = ros::Time::now() - start_query_position;
    if (since_query > ros::Duration(safe_landing_planner_->timeout_termination_)) {
      status_msg_.state = static_cast<int>(avoidance::MAV_STATE::MAV_STATE_FLIGHT_TERMINATION);
      publishSystemStatus();
    }
  }

  // Check if all information was received
  ros::Time now = ros::Time::now();
  ros::Duration since_last_algo = now - last_algo_time_;
  ros::Duration since_start = now - start_time_;
  checkFailsafe(since_last_algo, since_start);

  safe_landing_planner_->setPose(avoidance::toEigen(current_pose_.pose.position),
                                 avoidance::toEigen(current_pose_.pose.orientation));

  {
    std::lock_guard<std::mutex> transformed_cloud_guard(*(transformed_cloud_mutex_));

    safe_landing_planner_->runSafeLandingPlanner();
    cloud_transformed_ = false;
  }
  visualizer_.visualizeSafeLandingPlanner(*(safe_landing_planner_.get()), current_pose_.pose.position,
                                          previous_pose_.pose.position, rqt_param_config_);
  publishSerialGrid();
  last_algo_time_ = ros::Time::now();

  if (now - t_status_sent_ > ros::Duration(0.2)) publishSystemStatus();

  return;
}

void SafeLandingPlannerNode::checkFailsafe(ros::Duration since_last_algo, ros::Duration since_start) {
  ros::Duration timeout_termination = ros::Duration(safe_landing_planner_->timeout_termination_);
  ros::Duration timeout_critical = ros::Duration(safe_landing_planner_->timeout_critical_);

  if (since_last_algo > timeout_termination && since_start > timeout_termination) {
    status_msg_.state = static_cast<int>(avoidance::MAV_STATE::MAV_STATE_FLIGHT_TERMINATION);
  } else if (since_last_algo > timeout_critical && since_start > timeout_critical) {
    status_msg_.state = static_cast<int>(avoidance::MAV_STATE::MAV_STATE_CRITICAL);
  }
}

void SafeLandingPlannerNode::publishSystemStatus() {
  status_msg_.header.stamp = ros::Time::now();
  status_msg_.component = 196;  // MAV_COMPONENT_ID_AVOIDANCE we need to add a new component
  mavros_system_status_pub_.publish(status_msg_);
  t_status_sent_ = ros::Time::now();
}

void SafeLandingPlannerNode::publishSerialGrid() {
  static int grid_seq = 0;
  Grid prev_grid = safe_landing_planner_->getPreviousGrid();
  safe_landing_planner::SLPGridMsg grid;
  grid.header.frame_id = "local_origin";
  grid.header.seq = grid_seq;
  grid.grid_size = prev_grid.getGridSize();
  grid.cell_size = prev_grid.getCellSize();

  grid.mean.layout.dim.push_back(std_msgs::MultiArrayDimension());
  grid.mean.layout.dim.push_back(std_msgs::MultiArrayDimension());
  grid.mean.layout.dim[0].label = "height";
  grid.mean.layout.dim[0].size = prev_grid.mean_.cols();
  grid.mean.layout.dim[0].stride = prev_grid.mean_.rows() * prev_grid.mean_.cols();

  grid.mean.layout.dim[1].label = "width";
  grid.mean.layout.dim[1].size = prev_grid.mean_.rows();
  grid.mean.layout.dim[1].stride = prev_grid.mean_.rows();
  grid.mean.layout.data_offset = 0;

  grid.land.layout.dim.push_back(std_msgs::MultiArrayDimension());
  grid.land.layout.dim.push_back(std_msgs::MultiArrayDimension());
  grid.land.layout.dim[0].label = "height";
  grid.land.layout.dim[0].size = prev_grid.land_.cols();
  grid.land.layout.dim[0].stride = prev_grid.land_.rows() * prev_grid.land_.cols();

  grid.land.layout.dim[1].label = "width";
  grid.land.layout.dim[1].size = prev_grid.land_.rows();
  grid.land.layout.dim[1].stride = prev_grid.land_.rows();
  grid.land.layout.data_offset = 0;

  Eigen::MatrixXf variance = prev_grid.getVariance();
  grid.std_dev.layout.dim.push_back(std_msgs::MultiArrayDimension());
  grid.std_dev.layout.dim.push_back(std_msgs::MultiArrayDimension());
  grid.std_dev.layout.dim[0].label = "height";
  grid.std_dev.layout.dim[0].size = variance.cols();
  grid.std_dev.layout.dim[0].stride = variance.rows() * variance.cols();

  grid.std_dev.layout.dim[1].label = "width";
  grid.std_dev.layout.dim[1].size = variance.rows();
  grid.std_dev.layout.dim[1].stride = variance.rows();
  grid.std_dev.layout.data_offset = 0;

  Eigen::MatrixXi counter = prev_grid.getCounter();
  grid.counter.layout.dim.push_back(std_msgs::MultiArrayDimension());
  grid.counter.layout.dim.push_back(std_msgs::MultiArrayDimension());
  grid.counter.layout.dim[0].label = "height";
  grid.counter.layout.dim[0].size = counter.cols();
  grid.counter.layout.dim[0].stride = counter.rows() * counter.cols();

  grid.counter.layout.dim[1].label = "width";
  grid.counter.layout.dim[1].size = counter.rows();
  grid.counter.layout.dim[1].stride = counter.rows();
  grid.counter.layout.data_offset = 0;

  for (size_t i = 0; i < prev_grid.getRowColSize(); i++) {
    for (size_t j = 0; j < prev_grid.getRowColSize(); j++) {
      grid.mean.data.push_back(prev_grid.mean_(i, j));
      grid.land.data.push_back(prev_grid.land_(i, j));
      grid.std_dev.data.push_back(sqrtf(variance(i, j)));
      grid.counter.data.push_back(counter(i, j));
    }
  }
  Eigen::Vector2i pos_index = safe_landing_planner_->getPositionIndex();
  grid.curr_pos_index.x = static_cast<float>(pos_index.x());
  grid.curr_pos_index.y = static_cast<float>(pos_index.y());

  grid_pub_.publish(grid);
  grid_seq++;
}

void SafeLandingPlannerNode::pointCloudTransformThread() {
  while (!should_exit_) {
    {
      std::unique_lock<std::mutex> cloud_msg_lock(*(cloud_msg_mutex_));
      cloud_ready_cv_->wait(cloud_msg_lock);
    }
    while (cloud_transformed_ == false) {
      if (should_exit_) break;

      std::unique_ptr<std::lock_guard<std::mutex>> cloud_msg_lock(new std::lock_guard<std::mutex>(*(cloud_msg_mutex_)));
      if (tf_listener_.canTransform("local_origin", newest_cloud_msg_.header.frame_id,
                                    newest_cloud_msg_.header.stamp)) {
        try {
          pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
          // transform message to pcl type
          pcl::fromROSMsg(newest_cloud_msg_, pcl_cloud);
          cloud_msg_lock.reset();
          // remove nan padding
          std::vector<int> dummy_index;
          dummy_index.reserve(pcl_cloud.points.size());
          pcl::removeNaNFromPointCloud(pcl_cloud, pcl_cloud, dummy_index);

          // transform cloud to local_origin frame
          pcl_ros::transformPointCloud("local_origin", pcl_cloud, pcl_cloud, tf_listener_);

          std::lock_guard<std::mutex> transformed_cloud_guard(*(transformed_cloud_mutex_));
          cloud_transformed_ = true;
          safe_landing_planner_->cloud_ = std::move(pcl_cloud);
        } catch (tf::TransformException &ex) {
          ROS_ERROR("Received an exception trying to transform a pointcloud: %s", ex.what());
        }
      } else {
        cloud_msg_lock.reset();
        ros::Duration(0.001).sleep();
      }
    }
  }
}
}
