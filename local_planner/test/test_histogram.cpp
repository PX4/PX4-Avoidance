#include <gtest/gtest.h>

#include "../src/nodes/histogram.h"

using namespace avoidance;

TEST(TestsHistogram, emptyHistogram) {
  // GIVEN: a full histogram
  Histogram histogram = Histogram(ALPHA_RES);
  for (int e = 0; e < 180 / ALPHA_RES; e++) {
    for (int z = 0; z < 360 / ALPHA_RES; z++) {
      histogram.set_bin(e, z, 1.0f);
      histogram.set_dist(e, z, 5.0f);
      histogram.set_age(e, z, 1.0f);
    }
  }
  // WHEN: we empty the histogram
  histogram.setZero();

  // THEN: we expect an histogram with all cells set to zero
  for (int e = 0; e < 180 / ALPHA_RES; e++) {
    for (int z = 0; z < 360 / ALPHA_RES; z++) {
      EXPECT_FLOAT_EQ(0.0f, histogram.get_bin(e, z))
          << "Error at index (e, z): (" << e << ", " << z << ")";
      EXPECT_FLOAT_EQ(0.0f, histogram.get_bin(e, z))
          << "Error at index (e, z): (" << e << ", " << z << ")";
      EXPECT_FLOAT_EQ(0.0f, histogram.get_age(e, z))
          << "Error at index (e, z): (" << e << ", " << z << ")";
    }
  }
}

void set_histogram_cell(Histogram &h, int e, int z, float occ, float dist,
                        float age) {
  h.set_bin(e, z, occ);
  h.set_dist(e, z, dist);
  h.set_age(e, z, age);
}

TEST(TestsHistogram, upsampleHistogram) {
  // GIVEN: a histogram of resolition ALPHA_RES with 5 occupied cells
  Histogram histogram = Histogram(ALPHA_RES);
  Histogram histogram_truth = Histogram(ALPHA_RES / 2);
  std::vector<std::pair<int, int>> low_res_occupied;
  low_res_occupied.push_back(std::make_pair(0, 0));
  low_res_occupied.push_back(std::make_pair(0, 1));
  low_res_occupied.push_back(std::make_pair(0, 360 / ALPHA_RES - 1));
  low_res_occupied.push_back(std::make_pair(180 / ALPHA_RES - 1, 0));
  low_res_occupied.push_back(
      std::make_pair(180 / ALPHA_RES - 1, 360 / ALPHA_RES - 1));
  for (auto pair : low_res_occupied) {
    set_histogram_cell(histogram, pair.first, pair.second, 1.0f, 5.0f, 2.0f);
    set_histogram_cell(histogram_truth, 2 * pair.first, 2 * pair.second, 1.0f,
                       5.0f, 2.0f);
    set_histogram_cell(histogram_truth, 2 * pair.first, 2 * pair.second + 1,
                       1.0f, 5.0f, 2.0f);
    set_histogram_cell(histogram_truth, 2 * pair.first + 1, 2 * pair.second,
                       1.0f, 5.0f, 2.0f);
    set_histogram_cell(histogram_truth, 2 * pair.first + 1, 2 * pair.second + 1,
                       1.0f, 5.0f, 2.0f);
  }

  // WHEN: we upsample the histogram
  histogram.upsample();

  // THEN: we get a histogram of resolution ALPHA_RES/2 with 20 occupied cells
  for (int e = 0; e < 180 / (ALPHA_RES / 2); e++) {
    for (int z = 0; z < 360 / (ALPHA_RES / 2); z++) {
      EXPECT_FLOAT_EQ(histogram_truth.get_bin(e, z), histogram.get_bin(e, z))
          << "Error at index (e, z): (" << e << ", " << z << ")";
      EXPECT_FLOAT_EQ(histogram_truth.get_dist(e, z), histogram.get_dist(e, z))
          << "Error at index (e, z): (" << e << ", " << z << ")";
      EXPECT_FLOAT_EQ(histogram_truth.get_dist(e, z), histogram.get_dist(e, z))
          << "Error at index (e, z): (" << e << ", " << z << ")";
    }
  }
}

TEST(TestsHistogram, downsampleHistogram) {
  // GIVEN: a histogram of resolution ALPHA_RES with 10 occupied cells
  Histogram histogram = Histogram(ALPHA_RES);
  Histogram histogram_truth = Histogram(ALPHA_RES * 2);
  float d1 = 4.0f;
  float d2 = 3.0f;
  float d3 = 3.2f;
  float d4 = 3.9f;
  float a1 = 1.0f;

  set_histogram_cell(histogram, 0, 0, 1.0f, d1, a1);
  set_histogram_cell(histogram, 0, 1, 1.0f, d2, a1);
  set_histogram_cell(histogram, 1, 0, 1.0f, d3, a1);
  set_histogram_cell(histogram, 1, 1, 1.0f, d4, a1);
  set_histogram_cell(histogram_truth, 0, 0, 1.0f, (d1 + d2 + d3 + d4) / 4.0f,
                     a1 * 4.0f / 4.0f);

  set_histogram_cell(histogram, 180 / ALPHA_RES - 1, 0, 1.0f, d1, a1);
  set_histogram_cell(histogram, 180 / ALPHA_RES - 1, 1, 1.0f, d2, a1);
  set_histogram_cell(histogram_truth, 180 / (ALPHA_RES * 2) - 1, 0, 1.0f,
                     (d1 + d2) / 4.0f, a1 * 2.0f / 4.0f);

  set_histogram_cell(histogram, 180 / ALPHA_RES - 1, 360 / ALPHA_RES - 1, 1.0f,
                     d1, a1);
  set_histogram_cell(histogram_truth, 180 / (ALPHA_RES * 2) - 1,
                     360 / (ALPHA_RES * 2) - 1, 0.0f, d1 / 4.0f, a1 / 4.0f);

  set_histogram_cell(histogram, 0, 360 / ALPHA_RES - 1, 1.0f, d1, a1);
  set_histogram_cell(histogram, 1, 360 / ALPHA_RES - 1, 1.0f, d2, a1);
  set_histogram_cell(histogram, 0, 360 / ALPHA_RES - 2, 1.0f, d3, a1);
  set_histogram_cell(histogram_truth, 0, 360 / (ALPHA_RES * 2) - 1, 1.0f,
                     (d1 + d2 + d3) / 4.0f, a1 * 3.0f / 4.0f);

  // WHEN: we downsample the histogram
  histogram.downsample();

  // THEN: we get a histogram of resolution ALPHA_RES*2 with 3 occipied cells
  for (int e = 0; e < 180 / (ALPHA_RES * 2); e++) {
    for (int z = 0; z < 360 / (ALPHA_RES * 2); z++) {
      EXPECT_FLOAT_EQ(histogram_truth.get_bin(e, z), histogram.get_bin(e, z))
          << "Error at index (e, z): (" << e << ", " << z << ")";
      EXPECT_FLOAT_EQ(histogram_truth.get_dist(e, z), histogram.get_dist(e, z))
          << "Error at index (e, z): (" << e << ", " << z << ")";
      EXPECT_FLOAT_EQ(histogram_truth.get_age(e, z), histogram.get_age(e, z))
          << "Error at index (e, z): (" << e << ", " << z << ")";
    }
  }
}
