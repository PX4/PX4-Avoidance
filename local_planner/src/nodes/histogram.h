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

  inline void wrapIndex(int &x, int &y) const {
    while (x < 0) x += e_dim;
    while (x > e_dim - 1) x -= e_dim;
    while (y < 0) y += z_dim;
    while (y > z_dim - 1) y -= z_dim;
  }

 public:
  Histogram(const int res);
  ~Histogram();

  inline double get_bin(int x, int y) const {
    wrapIndex(x, y);
    return bin[x][y];
  }

  inline double get_age(int x, int y) const {
    wrapIndex(x, y);
    return age[x][y];
  }

  inline double get_dist(int x, int y) const {
    wrapIndex(x, y);
    return dist[x][y];
  }

  inline void set_bin(int x, int y, double value) { bin[x][y] = value; }
  inline void set_age(int x, int y, double value) { age[x][y] = value; }
  inline void set_dist(int x, int y, double value) { dist[x][y] = value; }

  void upsample();
  void downsample();
  void setZero();
};

#endif  // HISTOGRAM_H
