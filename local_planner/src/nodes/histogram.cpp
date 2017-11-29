#include "histogram.h"

Histogram::Histogram(const int res)
    : resolution { res },
      z_dim { 360 / resolution },
      e_dim { 180 / resolution } {

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

Histogram::~Histogram() {
}

double Histogram::get_bin(int x, int y) const {
  return bin[x][y];
}
double Histogram::get_age(int x, int y) const {
  return age[x][y];
}

double Histogram::get_dist(int x, int y) const {
  return dist[x][y];
}

void Histogram::set_bin(int x, int y, double value) {
  bin[x][y] = value;
}
void Histogram::set_age(int x, int y, double value) {
  age[x][y] = value;
}
void Histogram::set_dist(int x, int y, double value) {
  dist[x][y] = value;
}

void Histogram::upsample() {
  resolution = resolution / 2;
  z_dim = 2 * z_dim;
  e_dim = 2 * e_dim;
  std::vector < std::vector<double> > temp_bin;
  std::vector < std::vector<double> > temp_age;
  std::vector < std::vector<double> > temp_dist;
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

void Histogram::setZero() {
  for (int i = 0; i < e_dim; ++i) {
    for (int j = 0; j < z_dim; ++j) {
      bin[i][j] = 0.0;
      age[i][j] = 0.0;
      dist[i][j] = 0.0;
    }
  }
}
