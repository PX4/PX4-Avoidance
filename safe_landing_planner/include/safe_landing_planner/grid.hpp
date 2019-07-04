#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

namespace avoidance {

class Grid {
 public:
  Grid(const float grid_size, const float cell_size) : grid_size_(grid_size), cell_size_(cell_size) {
    resize(grid_size_, cell_size_);
  }
  ~Grid() = default;

  void reset() {
    mean_.fill(0.f);
    variance_.fill(0.f);
    counter_.fill(0);
    land_.fill(0);
  }

  void resize(float grid_size, float cell_size) {
    grid_size_ = grid_size;
    cell_size_ = cell_size;
    grid_row_col_size_ = static_cast<int>(std::ceil(grid_size_ / cell_size_));
    mean_.resize(grid_row_col_size_, grid_row_col_size_);
    variance_.resize(grid_row_col_size_, grid_row_col_size_);
    counter_.resize(grid_row_col_size_, grid_row_col_size_);
    land_.resize(grid_row_col_size_, grid_row_col_size_);
    reset();
  }

  void setMean(const Eigen::Vector2i &idx, float value) { mean_(idx.x(), idx.y()) = value; }
  void setVariance(const Eigen::Vector2i &idx, float value) { variance_(idx.x(), idx.y()) = value; }
  void increaseCounter(const Eigen::Vector2i &idx) { counter_(idx.x(), idx.y()) = counter_(idx.x(), idx.y()) + 1; }
  void setCounter(const Eigen::Vector2i &idx, int value) { counter_(idx.x(), idx.y()) = value; }

  Eigen::MatrixXf getMean() const { return mean_; }
  Eigen::MatrixXf getVariance() const { return variance_; }
  Eigen::MatrixXi getCounter() const { return counter_; }

  float getMean(const Eigen::Vector2i &idx) { return mean_(idx.x(), idx.y()); }
  float getVariance(const Eigen::Vector2i &idx) { return variance_(idx.x(), idx.y()); }
  int getCounter(const Eigen::Vector2i &idx) { return counter_(idx.x(), idx.y()); }
  int getRowColSize() const { return grid_row_col_size_; }
  float getGridSize() const { return grid_size_; }
  float getCellSize() const { return cell_size_; }

  void setFilterLimits(const Eigen::Vector3f &pos) {
    corner_min_.x() = pos.x() - grid_size_ / 2.f;
    corner_min_.y() = pos.y() - grid_size_ / 2.f;
    corner_max_.x() = pos.x() + grid_size_ / 2.f;
    corner_max_.y() = pos.y() + grid_size_ / 2.f;
  }

  void getGridLimits(Eigen::Vector2f &min, Eigen::Vector2f &max) const {
    min = corner_min_;
    max = corner_max_;
  }

  void combine(const Grid &prev_grid, float alpha) {
    mean_ = alpha * prev_grid.mean_ + (1.f - alpha) * mean_;
    variance_ = alpha * prev_grid.variance_ + (1.f - alpha) * variance_;
  }

  Eigen::MatrixXi land_;
  Eigen::MatrixXf mean_;

 private:
  Eigen::Vector2f corner_min_;
  Eigen::Vector2f corner_max_;
  Eigen::MatrixXf variance_;
  Eigen::MatrixXi counter_;

  float grid_size_;
  float cell_size_;
  int grid_row_col_size_;
};
}
