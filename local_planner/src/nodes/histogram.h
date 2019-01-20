#ifndef HISTOGRAM_H
#define HISTOGRAM_H

#include <math.h>
#include <vector>

namespace avoidance {

const int ALPHA_RES = 6;
const int GRID_LENGTH_Z = 360 / ALPHA_RES;
const int GRID_LENGTH_E = 180 / ALPHA_RES;

class Histogram {
  int resolution;
  int z_dim;
  int e_dim;
  std::vector<std::vector<float> > bin;
  std::vector<std::vector<float> > age;
  std::vector<std::vector<float> > dist;

  inline void wrapIndex(int &x, int &y) const {
    while (x < 0) x += e_dim;
    while (x > e_dim - 1) x -= e_dim;
    while (y < 0) y += z_dim;
    while (y > z_dim - 1) y -= z_dim;
  }

 public:
  Histogram(const int res);
  ~Histogram();

  inline float get_bin(int x, int y) const {
    wrapIndex(x, y);
    return bin[x][y];
  }

  inline float get_age(int x, int y) const {
    wrapIndex(x, y);
    return age[x][y];
  }

  inline float get_dist(int x, int y) const {
    wrapIndex(x, y);
    return dist[x][y];
  }

  inline void set_bin(int x, int y, float value) { bin[x][y] = value; }
  inline void set_age(int x, int y, float value) { age[x][y] = value; }
  inline void set_dist(int x, int y, float value) { dist[x][y] = value; }

  void upsample();
  void downsample();
  void setZero();
};
}

#endif  // HISTOGRAM_H
