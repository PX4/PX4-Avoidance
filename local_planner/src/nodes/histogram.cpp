#include "histogram.h"
namespace avoidance {
Histogram::Histogram(const int res)
    : resolution{res}, z_dim{360 / resolution}, e_dim{180 / resolution} {
  age.resize(e_dim, z_dim);
  dist.resize(e_dim, z_dim);
  setZero();
}

Histogram::~Histogram() {}

void Histogram::upsample() {
  resolution = resolution / 2;
  z_dim = 2 * z_dim;
  e_dim = 2 * e_dim;
  Eigen::MatrixXd temp_age;
  Eigen::MatrixXd temp_dist;
  temp_age.resize(e_dim, z_dim);
  temp_dist.resize(e_dim, z_dim);

  for (int i = 0; i < e_dim; ++i) {
    for (int j = 0; j < z_dim; ++j) {
      int i_lowres = floor(i / 2);
      int j_lowres = floor(j / 2);
      temp_age(i, j) = age(i_lowres, j_lowres);
      temp_dist(i, j) = dist(i_lowres, j_lowres);
    }
  }
  age = temp_age;
  dist = temp_dist;
}

void Histogram::downsample() {
  resolution = 2 * resolution;
  z_dim = z_dim / 2;
  e_dim = e_dim / 2;
  Eigen::MatrixXd temp_age;
  Eigen::MatrixXd temp_dist;
  temp_age.resize(e_dim, z_dim);
  temp_dist.resize(e_dim, z_dim);

  for (int i = 0; i < e_dim; ++i) {
    for (int j = 0; j < z_dim; ++j) {
      int i_high_res = 2 * i;
      int j_high_res = 2 * j;

      double mean_age =
          (age(i_high_res, j_high_res) + age(i_high_res + 1, j_high_res) +
           age(i_high_res, j_high_res + 1) +
           age(i_high_res + 1, j_high_res + 1)) /
          4.0;
      double mean_dist =
          (dist(i_high_res, j_high_res) + dist(i_high_res + 1, j_high_res) +
           dist(i_high_res, j_high_res + 1) +
           dist(i_high_res + 1, j_high_res + 1)) /
          4.0;

      temp_age(i, j) = mean_age;
      temp_dist(i, j) = mean_dist;
    }
  }
  age = temp_age;
  dist = temp_dist;
}

void Histogram::setZero() {
	  age.fill(0.0);
	  dist.fill(0.0);
}
}
