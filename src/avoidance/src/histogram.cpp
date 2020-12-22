#include "avoidance/histogram.h"
#include <stdexcept>

namespace avoidance {
Histogram::Histogram(const int res)
    : resolution_{res}, z_dim_{360 / resolution_}, e_dim_{180 / resolution_}, dist_(e_dim_, z_dim_) {
  setZero();
}

void Histogram::upsample() {
  if (resolution_ != ALPHA_RES * 2) {
    throw std::logic_error(
        "Invalid use of function upsample(). This function can only be used on a half resolution histogram.");
  }
  resolution_ = resolution_ / 2;
  z_dim_ = 2 * z_dim_;
  e_dim_ = 2 * e_dim_;
  Eigen::MatrixXf temp_dist(e_dim_, z_dim_);

  for (int i = 0; i < e_dim_; ++i) {
    for (int j = 0; j < z_dim_; ++j) {
      int i_lowres = floor(i / 2);
      int j_lowres = floor(j / 2);
      temp_dist(i, j) = dist_(i_lowres, j_lowres);
    }
  }
  dist_ = temp_dist;
}

void Histogram::downsample() {
  if (resolution_ != ALPHA_RES) {
    throw std::logic_error(
        "Invalid use of function downsample(). This function can only be used on a full resolution histogram.");
  }
  resolution_ = 2 * resolution_;
  z_dim_ = z_dim_ / 2;
  e_dim_ = e_dim_ / 2;
  Eigen::MatrixXf temp_dist(e_dim_, z_dim_);

  for (int i = 0; i < e_dim_; ++i) {
    for (int j = 0; j < z_dim_; ++j) {
      int i_high_res = 2 * i;
      int j_high_res = 2 * j;
      temp_dist(i, j) = dist_.block(i_high_res, j_high_res, 2, 2).mean();
    }
  }
  dist_ = temp_dist;
}

void Histogram::setZero() { dist_.fill(0.f); }

bool Histogram::isEmpty() const {
  int counter = 0;
  for (int e = 0; (e < e_dim_) && (0 == counter); e++) {
    for (int z = 0; (z < z_dim_) && (0 == counter); z++) {
      if (dist_(e, z) > FLT_MIN) {
        counter++;
      }
    }
  }
  return counter == 0;
}
}
