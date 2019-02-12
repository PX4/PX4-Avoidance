#include "local_planner/histogram.h"
#include <stdexcept>

namespace avoidance {
Histogram::Histogram(const int res)
    : resolution_{res},
      z_dim_{360 / resolution_},
      e_dim_{180 / resolution_},
      age_(e_dim_, z_dim_),
      dist_(e_dim_, z_dim_) {
  setZero();
}

Histogram::~Histogram() {}

void Histogram::upsample() {
  if (resolution_ != ALPHA_RES * 2) {
    throw std::logic_error(
        "Invalid use of function upsample(). This function can only be used on "
        "a half resolution histogram.");
  }
  resolution_ = resolution_ / 2;
  z_dim_ = 2 * z_dim_;
  e_dim_ = 2 * e_dim_;
  Eigen::MatrixXi temp_age(e_dim_, z_dim_);
  Eigen::MatrixXf temp_dist(e_dim_, z_dim_);

  for (int i = 0; i < e_dim_; ++i) {
    for (int j = 0; j < z_dim_; ++j) {
      int i_lowres = floor(i / 2);
      int j_lowres = floor(j / 2);
      temp_age(i, j) = age_(i_lowres, j_lowres);
      temp_dist(i, j) = dist_(i_lowres, j_lowres);
    }
  }
  age_ = temp_age;
  dist_ = temp_dist;
}

void Histogram::downsample() {
  if (resolution_ != ALPHA_RES) {
    throw std::logic_error(
        "Invalid use of function downsample(). This function can only be used "
        "on a full resolution histogram.");
  }
  resolution_ = 2 * resolution_;
  z_dim_ = z_dim_ / 2;
  e_dim_ = e_dim_ / 2;
  Eigen::MatrixXi temp_age(e_dim_, z_dim_);
  Eigen::MatrixXf temp_dist(e_dim_, z_dim_);

  for (int i = 0; i < e_dim_; ++i) {
    for (int j = 0; j < z_dim_; ++j) {
      int i_high_res = 2 * i;
      int j_high_res = 2 * j;
      temp_age(i, j) =
          static_cast<int>(age_.block(i_high_res, j_high_res, 2, 2).mean());
      temp_dist(i, j) = dist_.block(i_high_res, j_high_res, 2, 2).mean();
    }
  }
  age_ = temp_age;
  dist_ = temp_dist;
}

void Histogram::setZero() {
  age_.fill(0);
  dist_.fill(0.f);
}
}
