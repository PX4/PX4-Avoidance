#ifndef HISTOGRAM_H
#define HISTOGRAM_H

#include <math.h>
#include <vector>

const int ALPHA_RES = 6;
const int GRID_LENGTH_Z = 360 / ALPHA_RES;
const int GRID_LENGTH_E = 180 / ALPHA_RES;
const double H_FOV = 59.0;
const double V_FOV = 46.0;

class Histogram {
  int resolution;
  int z_dim;
  int e_dim;
  std::vector<std::vector<double> > bin;
  std::vector<std::vector<double> > age;
  std::vector<std::vector<double> > dist;
  void wrapIndex(int &x, int &y);

 public:
  Histogram(const int res);
  ~Histogram();

  double get_bin(int x, int y);
  double get_age(int x, int y);
  double get_dist(int x, int y);
  void set_bin(int x, int y, double value);
  void set_age(int x, int y, double value);
  void set_dist(int x, int y, double value);
  void upsample();
  void downsample();
  void setZero();
};

#endif  // HISTOGRAM_H
