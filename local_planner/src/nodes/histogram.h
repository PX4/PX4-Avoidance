#ifndef HISTOGRAM_H
#define HISTOGRAM_H

#include <vector>
#include <math.h>

const int ALPHA_RES = 6;
const int GRID_LENGTH_Z = 360/ALPHA_RES;
const int GRID_LENGTH_E = 180/ALPHA_RES;
const double H_FOV = 59.0;
const double V_FOV = 46.0;

class Histogram
{
  int resolution;
  int z_dim;
  int e_dim;
  std::vector<std::vector<double> > bin;
  std::vector<std::vector<double> > age;
  std::vector<std::vector<double> > dist;

 public:
  Histogram(const int res);
  ~Histogram();

  double get_bin(int x, int y) const;
  double get_age(int x, int y) const;
  double get_dist(int x, int y) const;
  void set_bin(int x, int y, double value);
  void set_age(int x, int y, double value);
  void set_dist(int x, int y, double value);
  void upsample();
  void downsample();
  void setZero();
};

#endif // HISTOGRAM_H
