// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <random>

#include <gtest/gtest.h>

#include "core/Geometry.h"

using namespace dv;


/**
 * Tests that the file name recorded by the exception is correct.
 */
TEST(GeometryTest, planeFit) {
  std::vector<cv::Point3f> points;

  /// 100 points on a the xz plane.
  for (int i = 0; i < 10; ++i) {
    for (int j = 0; j < 10; ++j) {
      points.emplace_back(i - 5.0f, 0.0f, j - 5.0f);
    }
  }

  // 300 random points.
  {
    std::seed_seq seed{0};
    std::mt19937 gen{seed};
    std::uniform_real_distribution<> xz(-7.0f, 7.0f);
    std::uniform_real_distribution<> y(-7.0f, 7.0f);
    for (size_t i = 0; i < 300; ++i) {
      points.emplace_back(xz(gen), y(gen), xz(gen));
    }
  }

  // Fit the plane.
  auto plane = planeFit(points, 1000, 100, 0.1f);
  EXPECT_NEAR(0.0f, plane.nx, 0.1f);
  EXPECT_NEAR(1.0f, std::abs(plane.ny), 0.1f);
  EXPECT_NEAR(0.0f, plane.nz, 0.1f);
  EXPECT_NEAR(0.0f, plane.d, 0.1f);
}
