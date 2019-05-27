#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

namespace avoidance {

class Grid {
public:
  Grid(const float grid_size, const float cell_size) :
    grid_size_(grid_size),
    cell_size_(cell_size),
    mean_(static_cast<int>(std::ceil(grid_size/cell_size)), static_cast<int>(std::ceil(grid_size/cell_size))),
    variance_(static_cast<int>(std::ceil(grid_size/cell_size)), static_cast<int>(std::ceil(grid_size/cell_size))),
    counter_(static_cast<int>(std::ceil(grid_size/cell_size)), static_cast<int>(std::ceil(grid_size/cell_size))),
    land_(static_cast<int>(std::ceil(grid_size/cell_size)), static_cast<int>(std::ceil(grid_size/cell_size)))
    {
      reset();
    }
  ~Grid() = default;

  void reset() {
    mean_.fill(0.f);
    variance_.fill(0.f);
    counter_.fill(0);
    land_.fill(0);
  }

  void resize() {
    mean_.resize(static_cast<int>(std::ceil(grid_size_/cell_size_)), static_cast<int>(std::ceil(grid_size_/cell_size_)));
    variance_.resize(static_cast<int>(std::ceil(grid_size_/cell_size_)), static_cast<int>(std::ceil(grid_size_/cell_size_)));
    counter_.resize(static_cast<int>(std::ceil(grid_size_/cell_size_)), static_cast<int>(std::ceil(grid_size_/cell_size_)));
    land_.resize(static_cast<int>(std::ceil(grid_size_/cell_size_)), static_cast<int>(std::ceil(grid_size_/cell_size_)));
  }

  void setMean(Eigen::Vector2i &idx, float value) {
    mean_(idx.x(), idx.y()) = value;
  }
  void setVariance(Eigen::Vector2i &idx, float value) {
    variance_(idx.x(), idx.y()) = value;
  }
  void increaseCounter(Eigen::Vector2i &idx) {
    counter_(idx.x(), idx.y()) = counter_(idx.x(), idx.y()) + 1;
  }

  Eigen::MatrixXf getMean() const { return mean_;}
  Eigen::MatrixXf getVariance() const { return variance_;}
  Eigen::MatrixXi getCounter() const { return counter_;}

  float getMean(Eigen::Vector2i &idx) {return mean_(idx.x(), idx.y());}
  float getVariance(Eigen::Vector2i &idx) {return variance_(idx.x(), idx.y());}
  int getCounter(Eigen::Vector2i &idx) {return counter_(idx.x(), idx.y()); }

  void setFilterLimits(Eigen::Vector3f &pos) {
    corner_min_.x() = pos.x() - grid_size_ / 2.f;
    corner_min_.y() = pos.y() - grid_size_ / 2.f;
    corner_max_.x() = pos.x() + grid_size_ / 2.f;
    corner_max_.y() = pos.y() + grid_size_ / 2.f;
  }

  void getGridLimits(Eigen::Vector2f &min, Eigen::Vector2f &max) const {
    min = corner_min_;
    max = corner_max_;
  }

  float grid_size_;
  float cell_size_;
  Eigen::MatrixXi land_;
  Eigen::MatrixXf mean_;

private:
  Eigen::Vector2f corner_min_;
  Eigen::Vector2f corner_max_;
  Eigen::MatrixXf variance_;
  Eigen::MatrixXi counter_;
};

}
