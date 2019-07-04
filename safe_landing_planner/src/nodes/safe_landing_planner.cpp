#include "safe_landing_planner/safe_landing_planner.hpp"
#include "avoidance/common.h"

namespace avoidance {

void SafeLandingPlanner::runSafeLandingPlanner() {
  if (size_update_) {
    grid_.resize(grid_size_, cell_size_);
    previous_grid_.resize(grid_size_, cell_size_);
    n_lines_padding_ = smoothing_size_;
    size_update_ = false;
  }
  if (!play_rosbag_) {
    processPointcloud();
  } else {
    processRawGrid();
  }

  // low pass filter on grid mean and variance
  grid_.combine(previous_grid_, alpha_);
  isLandingPossible();
}

void SafeLandingPlanner::processPointcloud() {
  std::swap(previous_grid_, grid_);
  grid_.setFilterLimits(position_);
  grid_seq_ += 1;
  grid_.reset();
  visualization_cloud_.header = cloud_.header;
  visualization_cloud_.points.clear();
  ROS_INFO("Input cloud size %lu ", cloud_.points.size());
  for (const pcl::PointXYZ& xyz : cloud_) {
    if (!std::isnan(xyz.x) && !std::isnan(xyz.y) && !std::isnan(xyz.z)) {
      // check if point is inside the grid
      if (isInsideGrid(xyz.x, xyz.y)) {
        // calculate the cell indexes to which the points maps to
        Eigen::Vector2i grid_index = computeGridIndexes(xyz.x, xyz.y);
        float prev_mean = grid_.getMean(grid_index);
        float prev_variance = grid_.getVariance(grid_index);
        grid_.increaseCounter(grid_index);
        std::pair<float, float> mean_variance = computeOnlineMeanVariance(
            prev_mean, prev_variance, xyz.z, static_cast<float>(grid_.getCounter(grid_index)));
        grid_.setMean(grid_index, mean_variance.first);
        grid_.setVariance(grid_index, mean_variance.second);

        // cloud for visualization of the binning
        visualization_cloud_.points.push_back(
            avoidance::toXYZI(xyz, grid_index.x() * (grid_size_ / cell_size_) + grid_index.y()));
      }
    }
  }
}

void SafeLandingPlanner::processRawGrid() {
  grid_seq_ = raw_grid_.header.seq;
  std::swap(previous_grid_, grid_);
  grid_.reset();
  grid_.setFilterLimits(position_);

  if (grid_.getGridSize() != raw_grid_.grid_size || grid_.getCellSize() != raw_grid_.cell_size) {
    grid_.resize(raw_grid_.grid_size, raw_grid_.cell_size);
  }

  for (int i = 0; i < raw_grid_.mean.layout.dim[0].size; i++) {
    for (int j = 0; j < raw_grid_.mean.layout.dim[1].size; j++) {
      Eigen::Vector2i grid_index = Eigen::Vector2i(i, j);
      grid_.setMean(grid_index, raw_grid_.mean.data[raw_grid_.mean.layout.dim[1].size * i + j]);
      grid_.setVariance(grid_index, powf(raw_grid_.std_dev.data[raw_grid_.std_dev.layout.dim[1].size * i + j], 2));
      grid_.setCounter(grid_index, raw_grid_.counter.data[raw_grid_.counter.layout.dim[1].size * i + j]);
    }
  }
}

void SafeLandingPlanner::isLandingPossible() {
  int size = grid_.getRowColSize();
  // decide if it's possible to land in each cell based on variance and numeber
  // of points
  for (int i = 0; i < size; i++) {
    for (int j = 0; j < size; j++) {
      Eigen::Vector2i idx(i, j);
      if (grid_.getCounter(idx) < n_points_thr_ || sqrtf(grid_.getVariance(idx)) > std_dev_thr_) {
        grid_.land_(i, j) = 0;
      } else {
        grid_.land_(i, j) = 1;
      }
    }
  }

  // if grid smoothing enabled
  if (n_lines_padding_ > 0) {
    Eigen::MatrixXi land_padded(size + 2 * n_lines_padding_, size + 2 * n_lines_padding_);
    Eigen::MatrixXi land_accumulator(size + 2 * n_lines_padding_, size + 2 * n_lines_padding_);
    land_padded.fill(0);
    land_accumulator.fill(0);
    Eigen::MatrixXf mean_padded(size + 2 * n_lines_padding_, size + 2 * n_lines_padding_);
    mean_padded.fill(0.f);
    Eigen::MatrixXi mean_accumulator(size + 2 * n_lines_padding_, size + 2 * n_lines_padding_);
    mean_accumulator.fill(0);

    // copy grid_.land_ into the center of the padded matrix
    land_padded.block(n_lines_padding_, n_lines_padding_, grid_.land_.rows(), grid_.land_.cols()) = grid_.land_;
    mean_padded.block(n_lines_padding_, n_lines_padding_, grid_.mean_.rows(), grid_.mean_.cols()) = grid_.mean_;

    for (int i = n_lines_padding_; i < land_padded.rows() - n_lines_padding_; i++) {
      for (int j = n_lines_padding_; j < land_padded.cols() - n_lines_padding_; j++) {
        for (int k = -n_lines_padding_; k <= n_lines_padding_; k++) {
          for (int t = -n_lines_padding_; t <= n_lines_padding_; t++) {
            land_accumulator(i, j) += land_padded(i + k, j + t);
            float mean_diff = std::abs(mean_padded(i, j) - mean_padded(i + k, j + t));
            if (mean_diff > mean_diff_thr_) {
              mean_accumulator(i, j) += 1;
            }
          }
        }
      }
    }

    // threshold each cell based on the number of landable cells in the
    // neighborhood
    land_accumulator = (land_accumulator.array() <= min_n_land_cells_).select(0, land_accumulator);
    land_accumulator = (land_accumulator.array() > min_n_land_cells_).select(1, land_accumulator);

    // threshold each cell based on the number of cells in the neighborhood with
    // mean value difference greater than mean_diff_thr_
    mean_accumulator = (mean_accumulator.array() <= max_n_mean_diff_cells_).select(1, mean_accumulator);
    mean_accumulator = (mean_accumulator.array() > max_n_mean_diff_cells_).select(0, mean_accumulator);

    // logical AND between mean_accumulator and land_accumulator
    land_accumulator = mean_accumulator.cwiseProduct(land_accumulator);

    // copy back into grid.land_
    grid_.land_.block(0, 0, grid_.land_.rows(), grid_.land_.cols()) =
        land_accumulator.block(n_lines_padding_, n_lines_padding_, grid_.land_.rows(), grid_.land_.cols());
  }
  pos_index_ = computeGridIndexes(position_.x(), position_.y());
}

void SafeLandingPlanner::setPose(const Eigen::Vector3f& pos, const Eigen::Quaternionf& q) { position_ = pos; }

bool SafeLandingPlanner::isInsideGrid(float x, float y) {
  Eigen::Vector2f grid_min, grid_max;
  grid_.getGridLimits(grid_min, grid_max);
  return x < grid_max.x() && x > grid_min.x() && y < grid_max.y() && y > grid_min.y();
}

Eigen::Vector2i SafeLandingPlanner::computeGridIndexes(float x, float y) {
  Eigen::Vector2f grid_min, grid_max;
  grid_.getGridLimits(grid_min, grid_max);
  Eigen::Vector2i idx(static_cast<int>(std::floor((x - grid_min.x()) / grid_.getCellSize())),
                      static_cast<int>(std::floor((y - grid_min.y()) / grid_.getCellSize())));
  return idx;
}

std::pair<float, float> SafeLandingPlanner::computeOnlineMeanVariance(float prev_mean, float prev_variance,
                                                                      float new_value, float seq) {
  std::pair<float, float> pair(NAN, NAN);
  pair.first = (prev_mean * (seq - 1) + new_value) / (seq);

  // Welford's algorithm
  // https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
  float delta = new_value - prev_mean;
  float delta2 = new_value - pair.first;
  float prev_M2 = 0;
  if ((seq - 1) >= 0) {
    prev_M2 = prev_variance * (seq - 1);
  }
  float M2 = prev_M2 + delta * delta2;
  if (seq > 0) {
    pair.second = M2 / seq;
  }
  return pair;
}

// set parameters changed by dynamic rconfigure
void SafeLandingPlanner::dynamicReconfigureSetParams(const safe_landing_planner::SafeLandingPlannerNodeConfig& config,
                                                     uint32_t level) {
  size_update_ = false;
  n_points_thr_ = static_cast<float>(config.n_points_threshold);
  std_dev_thr_ = static_cast<float>(config.std_dev_threshold);
  smoothing_size_ = config.smoothing_size;
  mean_diff_thr_ = static_cast<float>(config.mean_diff_thr);
  max_n_mean_diff_cells_ = config.max_n_mean_diff_cells;
  grid_size_ = static_cast<float>(config.grid_size);
  cell_size_ = static_cast<float>(config.cell_size);
  alpha_ = static_cast<float>(config.alpha);
  timeout_critical_ = config.timeout_critical;
  timeout_termination_ = config.timeout_termination;
  min_n_land_cells_ = config.min_n_land_cells;
  if ((grid_.getGridSize() != grid_size_) || (grid_.getCellSize() != cell_size_) ||
      (n_lines_padding_ != smoothing_size_)) {
    size_update_ = true;
  }
}
}
