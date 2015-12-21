// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <random>

#include <gtest/gtest.h>

#include "core/Geometry.h"

using namespace dv;


/**
 * planeFit should detect a plane with 100 in a cloud 300 more noisy points.
 */
TEST(GeometryTest, PlaneFit) {
  std::vector<cv::Point3f> points;

  /// 100 points on a the xz plane at distance 2.0f.
  for (int i = 0; i < 10; ++i) {
    for (int j = 0; j < 10; ++j) {
      points.emplace_back(i - 5.0f, 2.0f, j - 5.0f);
    }
  }

  // 300 random points.
  {
    std::seed_seq seed{0};
    std::mt19937 gen{seed};
    std::uniform_real_distribution<> xz(-5.0f, 9.0f);
    std::uniform_real_distribution<> y(-5.0f, 9.0f);
    for (size_t i = 0; i < 300; ++i) {
      points.emplace_back(xz(gen), y(gen), xz(gen));
    }
  }

  // Fit the plane. It must be parallel to xz, at distance 2 from origin.
  auto plane = planeFit(points, 1000, 100, 1e-5f);
  EXPECT_NEAR(0.000000f, plane.nx, 1e-5f);
  EXPECT_NEAR(0.454166f, plane.ny, 1e-5f);
  EXPECT_NEAR(0.000000f, plane.nz, 1e-5f);
  EXPECT_NEAR(0.890916f, plane.d,  1e-5f);
}

/**
 * transformPlane should correctly rotate points on a plane.
 */
TEST(GeometryTest, TransformPlane) {
  std::vector<cv::Point3f> points;

  // 100 points on a plane tilted 45 degrees around z.
  for (int i = -10; i <= 10; ++i) {
    for (int j = -10; j <= 10; ++j) {
      const float x = i + j;
      const float y = i + j;
      const float z = i - j;
      points.emplace_back(
          0.5f + x * 0.005f,
          0.0f + y * 0.005f,
          0.5f + z * 0.005f);
    }
  }

  // Rotate the points to the xy plane and move them to origin.
  auto rotated = transformPlane(points, {0.0f, 0.0f, 1.0f});
  for (const auto &point : rotated) {
    EXPECT_NEAR(0.0f, point.z - rotated[0].z, 1e-5f);
  }

  // Make sure relative distances are the same.
  for (size_t i = 0; i < points.size(); ++i) {
    for (size_t j = i + 1; j < points.size(); ++j) {
      const auto &s0 = points[i];
      const auto &e0 = points[j];
      const auto &s1 = rotated[i];
      const auto &e1 = rotated[j];

      const float d0 = std::sqrt((s0 - e0).dot(s0 - e0));
      const float d1 = std::sqrt((s1 - e1).dot(s1 - e1));

      EXPECT_NEAR(d0, d1, 1e-5f);
    }
  }
}

