#ifndef HISTOGRAM_H
#define HISTOGRAM_H

#include <float.h>
#include <math.h>
#include <Eigen/Dense>
#include <vector>

namespace avoidance {

// Be very careful choosing the resolution! Valid resolutions must fullfill:
// 180 % (2 * ALPHA_RES) = 0
// Examples of valid resolution values: 1, 3, 5, 6, 10, 15, 18, 30, 45, 60
const int ALPHA_RES = 6;
const int GRID_LENGTH_Z = 360 / ALPHA_RES;
const int GRID_LENGTH_E = 180 / ALPHA_RES;

class Histogram {
  int resolution_;
  int z_dim_;
  int e_dim_;
  Eigen::MatrixXf dist_;

  /**
  * @brief     wraps elevation and azimuth indeces around the histogram
  * @param     x, elevation angle index
  * @param     y, azimuth angle index
  **/
  inline void wrapIndex(int &x, int &y) const {
    x = x % e_dim_;
    if (x < 0) x += e_dim_;
    y = y % z_dim_;
    if (y < 0) y += z_dim_;
  }

 public:
  Histogram(const int res);
  ~Histogram() = default;

  /**
  * @brief     getter method for histogram cell distance
  * @param[in] x, elevation angle index
  * @param[in] y, azimuth angle index
  * @returns   distance to the vehicle of obstacle mapped to (x, y) cell [m]
  **/
  inline float get_dist(int x, int y) const {
    wrapIndex(x, y);
    return dist_(x, y);
  }

  /**
  * @brief     setter method for histogram cell distance
  * @param[in] x, elevation angle index
  * @param[in] y, azimuth angle index
  * @param[in] value, distance to the vehicle of obstacle mapped to (x, y) cell
  *[m]
  **/
  inline void set_dist(int x, int y, float value) { dist_(x, y) = value; }

  /**
  * @brief     Compute the upsampled version of the histogram
  * @param[in] This object. Needs to be a histogram the larger bin size
  *(ALPHA_RES * 2)
  * @details   The histogram is upsampled to get the same histogram at regular
  *bin size (ALPHA_RES).
  *            This means the histogram matrix will be double the size in each
  *dimension
  * @returns   Modifies the object it is called from to have regular resolution
  * @warning   Can only be called from a large bin size histogram
  **/
  void upsample();

  /**
  * @brief     Compute the downsampled version of the histogram
  * @param[in] this object. Needs to be a histogram with regular bin size
  *(ALPHA_RES)
  * @details   The histogram is downsampled to get the same histogram at larger
  *bin size (ALPHA_RES/2).
  *            This means the histogram matrix will be half the size in each
  *dimension
  * @returns   Modifies the object it is called from to have larger bins
  * @warning   Can only be called from a regular bin size histogram
  **/
  void downsample();

  /**
  * @brief     resets all histogram cells age and distance to zero
  **/
  void setZero();

  /**
  * @brief     determines whether the histogram is empty (distance layer
  *            contains no distance bigger than zero)
  * @returns   whether histogram is empty
  **/
  bool isEmpty() const;
};
}

#endif  // HISTOGRAM_H
