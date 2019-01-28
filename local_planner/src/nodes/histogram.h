#ifndef HISTOGRAM_H
#define HISTOGRAM_H

#include <math.h>
#include <Eigen/Dense>
#include <vector>

namespace avoidance {

// Be very careful choosing the resolution! Valid resolutions must fullfill: 180
// % (2 * ALPHA_RES) = 0
// Examples of valid resolution values: 1, 3, 5, 6, 10, 15, 18, 30, 45, 60
const int ALPHA_RES = 6;
const int GRID_LENGTH_Z = 360 / ALPHA_RES;
const int GRID_LENGTH_E = 180 / ALPHA_RES;
const double H_FOV = 59.0;
const double V_FOV = 46.0;

class Histogram {
  int resolution;
  int z_dim;
  int e_dim;
  Eigen::MatrixXd age;
  Eigen::MatrixXd dist;

  inline void wrapIndex(int &x, int &y) const {
    while (x < 0) x += e_dim;
    while (x > e_dim - 1) x -= e_dim;
    while (y < 0) y += z_dim;
    while (y > z_dim - 1) y -= z_dim;
  }

 public:
  Histogram(const int res);
  ~Histogram();

  inline double get_age(int x, int y) const {
    wrapIndex(x, y);
    return age(x, y);
  }

  inline double get_dist(int x, int y) const {
    wrapIndex(x, y);
    return dist(x, y);
  }

  inline void set_age(int x, int y, double value) { age(x, y) = value; }
  inline void set_dist(int x, int y, double value) { dist(x, y) = value; }

  /**
  * @brief     Compute the upsampled version of the histogram
  * @param[in] This object. Needs to be a histogram the larger bin size (ALPHA_RES * 2)
  * @details   The histogram is upsampled to get the same histogram at regular bin size (ALPHA_RES).
  *            This means the histogram matrix will be double the size in each dimension
  * @returns   Modifies the object it is called from to have regular resolution
  * @warning   Can only be called from a large bin size histogram
  **/
  void upsample();

  /**
  * @brief     Compute the downsampled version of the histogram
  * @param[in] this object. Needs to be a histogram with regular bin size (ALPHA_RES)
  * @details   The histogram is downsampled to get the same histogram at larger bin size (ALPHA_RES/2).
  *            This means the histogram matrix will be half the size in each dimension
  * @returns   Modifies the object it is called from to have larger bins
  * @warning   Can only be called from a regular bin size histogram
  **/
  void downsample();
  void setZero();
};
}

#endif  // HISTOGRAM_H
