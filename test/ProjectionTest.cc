// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <iostream>

#include "core/Projection.h"

using namespace dv;


/**
 * Check a point from the origin of a 1x1 depth map is correctly mapped to 3D
 * coordinates.
 */
TEST(ProjectionTest, InvertTestOrigin) {
  // Parameters of the camera.
  float params[9] {
    0.5f, 0.0f, 0.5f,
    0.0f, 0.5f, 0.5f,
    0.0f, 0.0f, 1.0f
  };
  const cv::Mat cameraParams(3, 3, CV_32FC1, &params);

  // Radial distortion.
  const float k1 = 0.5f;
  const float k2 = 0.5f;

  // Depth image (z coordinate).
  const float z = 5550.0f;

  // Get 3D coordinates from the depth image.
  const auto point3d = projection::map3D(cameraParams, z, 0, 0);

  // Calculate the coordinates of the projection.
  const auto projection = projection::project(cameraParams, k1, k2, point3d);

  // Check that the projection is the same as the original point.
  ASSERT_EQ(projection.x, 0);
  ASSERT_EQ(projection.y, 0);
  ASSERT_NEAR(z, point3d.z, 0.1f);
}

/**
 * Check that if we take a (random) point from the depth image, retrieve its
 * 3D coordinates, and then project it, we get the original UV coordinates.
 */
TEST(ProjectionTest, InvertTestRandomPoint) {
  // Parameters of the camera.
  float params[9] {
    0.1f, 0.0f, 0.03f,
    0.0f, 0.3f, 0.05f,
    0.0f, 0.0f, 1.0f
  };
  const cv::Mat cameraParams(3, 3, CV_32FC1, &params);

  // Radial distortion.
  const float k1 = 0.5f;
  const float k2 = 0.5f;

  // Get 3D coordinates from the depth image.
  const auto point3d = projection::map3D(cameraParams, 1003.0f, 2, 1);

  // Calculate the coordinates of the projection.
  const auto projection = projection::project(cameraParams, k1, k2, point3d);

  // Check that the projection is the same as the original point.
  // OpenCV rounds down when constructin cv::Point2i from float values.
  // Thus, 1.9 computed in projection::project gets rounded to 1, rather than 2.
  ASSERT_EQ(projection.x, 1);
  ASSERT_EQ(projection.y, 1);
}

