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
TEST(GeometryTest, planeFit) {
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
TEST(GeometryTest, transformPlane) {
  std::vector<cv::Point3f> points;

  // 25 points on a the xz plane at distance 2.0f.
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 5; ++j) {
      points.emplace_back(i - 2.5f, 2.0f, j - 2.5f);
    }
  }

  // Rotate the points to the xy plane and move them to origin.
  auto rotated = transformPlane(points, {0.0f, 0.0f, 1.0f});
  for (const auto &point : rotated) {
    EXPECT_NEAR(0.0f, point.z - rotated[0].z, 1e-5f);
  }
}


/**
 * transformPlane throws if points are collinear.
 */
TEST(GeometryTest, transformPlaneCollinear) {
  std::vector<cv::Point3f> points;

  // 10 points on a line.
  for (int i = 0; i < 10; ++i) {
    points.emplace_back(i, 0, i);
  }

  EXPECT_ANY_THROW(transformPlane(points, {0.0f, 1.0f, 0.0f}));
}
