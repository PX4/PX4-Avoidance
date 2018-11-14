#include "histogram.h"

Histogram::Histogram(const int res)
    : resolution{res}, z_dim{360 / resolution}, e_dim{180 / resolution} {
  bin.resize(e_dim);
  age.resize(e_dim);
  dist.resize(e_dim);
  for (int i = 0; i < e_dim; ++i) {
    bin[i].resize(z_dim);
    age[i].resize(z_dim);
    dist[i].resize(z_dim);
  }
  setZero();
}

Histogram::~Histogram() {}

void Histogram::upsample() {
  resolution = resolution / 2;
  z_dim = 2 * z_dim;
  e_dim = 2 * e_dim;
  std::vector<std::vector<double> > temp_bin;
  std::vector<std::vector<double> > temp_age;
  std::vector<std::vector<double> > temp_dist;
  temp_bin.resize(e_dim);
  temp_age.resize(e_dim);
  temp_dist.resize(e_dim);
  for (int i = 0; i < e_dim; ++i) {
    temp_bin[i].resize(z_dim);
    temp_age[i].resize(z_dim);
    temp_dist[i].resize(z_dim);
  }
  for (int i = 0; i < e_dim; ++i) {
    for (int j = 0; j < z_dim; ++j) {
      int i_lowres = floor(i / 2);
      int j_lowres = floor(j / 2);
      temp_bin[i][j] = bin[i_lowres][j_lowres];
      temp_age[i][j] = age[i_lowres][j_lowres];
      temp_dist[i][j] = dist[i_lowres][j_lowres];
    }
  }
  bin = temp_bin;
  age = temp_age;
  dist = temp_dist;
}

void Histogram::downsample() {
  resolution = 2 * resolution;
  z_dim = z_dim / 2;
  e_dim = e_dim / 2;
  std::vector<std::vector<double> > temp_bin;
  std::vector<std::vector<double> > temp_age;
  std::vector<std::vector<double> > temp_dist;
  temp_bin.resize(e_dim);
  temp_age.resize(e_dim);
  temp_dist.resize(e_dim);
  for (int i = 0; i < e_dim; ++i) {
    temp_bin[i].resize(z_dim);
    temp_age[i].resize(z_dim);
    temp_dist[i].resize(z_dim);
  }
  for (int i = 0; i < e_dim; ++i) {
    for (int j = 0; j < z_dim; ++j) {
      int i_high_res = 2 * i;
      int j_high_res = 2 * j;

      double mean_bin =
          (bin[i_high_res][j_high_res] + bin[i_high_res + 1][j_high_res] +
           bin[i_high_res][j_high_res + 1] +
           bin[i_high_res + 1][j_high_res + 1]) /
          4.0;
      double mean_age =
          (age[i_high_res][j_high_res] + age[i_high_res + 1][j_high_res] +
           age[i_high_res][j_high_res + 1] +
           age[i_high_res + 1][j_high_res + 1]) /
          4.0;
      double mean_dist =
          (dist[i_high_res][j_high_res] + dist[i_high_res + 1][j_high_res] +
           dist[i_high_res][j_high_res + 1] +
           dist[i_high_res + 1][j_high_res + 1]) /
          4.0;

      if (mean_bin >= 0.5) {
        temp_bin[i][j] = 1.0;
      } else {
        temp_bin[i][j] = 0.0;
      }
      temp_age[i][j] = mean_age;
      temp_dist[i][j] = mean_dist;
    }
  }
  bin = temp_bin;
  age = temp_age;
  dist = temp_dist;
}

void Histogram::setZero() {
  for (int i = 0; i < e_dim; ++i) {
    for (int j = 0; j < z_dim; ++j) {
      bin[i][j] = 0.0;
      age[i][j] = 0.0;
      dist[i][j] = 0.0;
    }
  }
}
