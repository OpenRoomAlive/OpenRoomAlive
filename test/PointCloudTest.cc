// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <gtest/gtest.h>

#include "master/PointCloud.h"
#include "master/ProCamSystem.h"

using namespace dv;
using namespace dv::master;

constexpr size_t kRows = 424;
constexpr size_t kCols = 512;

/**
 * Test which verifies that a 3D point cloud is rendered properly for a white
 * planar surface.
 */
TEST(PointCloudTest, SinglePlane) {
  // Set the depth camera parameters used for test. Intrinsic parameters are set
  // so that when converting prom 2D to 3D point (using the depth information)
  // the magnitude of x and y coordinates does not change.
  CameraModel depthCamera;

  cv::Mat irCamParams(3, 3, CV_32FC1);
  irCamParams.at<float>(0, 0) = 1.0f;
  irCamParams.at<float>(1, 1) = 1.0f;
  irCamParams.at<float>(0, 2) = 0.0f;
  irCamParams.at<float>(1, 2) = 0.0f;
  irCamParams.at<float>(2, 2) = 1.0f;

  cv::Mat irDist(5, 1, CV_32FC1);
  irDist.at<float>(0, 0) = 0.1f;
  irDist.at<float>(1, 0) = 0.1f;
  irDist.at<float>(2, 0) = 0.1f;
  irDist.at<float>(3, 0) = 0.1f;
  irDist.at<float>(4, 0) = 0.1f;

  depthCamera.calib = irCamParams;
  depthCamera.dist = irDist;

  cv::Mat depthMap(kRows, kCols, cv::DataType<float>::type);
  cv::Mat colorMap(kRows, kCols, CV_32FC1);

  // Set the distance of all points to 1 meter and colour to white.
  for (size_t i = 0; i < kRows; i++) {
    for (size_t j = 0; j < kCols; j++) {
      // Set the depth map - uniform depth of 1 meter.
      depthMap.at<float>(i, j) = 1000.0f;

      // Set the color map - set all pixels to white.
      cv::Vec4b &pixel = colorMap.at<cv::Vec4b>(i,j);
      pixel[0] = 255;
      pixel[1] = 255;
      pixel[2] = 255;
      pixel[3] = 1;
    }
  }

  // Set the extrinsic parameters.
  CameraPose pose;

  pose.tvec = cv::Mat(3, 1, cv::DataType<double>::type);
  pose.tvec.at<double>(0, 0) = 0.0f;
  pose.tvec.at<double>(1, 0) = 0.0f;
  pose.tvec.at<double>(2, 0) = 0.01f;

  pose.rvec = cv::Mat(3, 1, cv::DataType<double>::type);
  cv::Mat rotMat = cv::Mat::eye(3, 3, cv::DataType<double>::type);
  cv::Rodrigues(rotMat, pose.rvec);

  // Create a point cloud.
  std::vector<ConnectionID> ids;
  auto sys = std::shared_ptr<ProCamSystem>(new ProCamSystem());

  PointCloud pointCloud(ids, sys);
  pointCloud.addView(0, depthMap, colorMap, depthCamera, pose);

  std::vector<Vertex> points = pointCloud.getPoints();

  // Check that the magnitude of x and y coordinates does not change and that
  // the z coordinate is equal to the depth increased by translation.
  size_t k = 0;
  for (size_t y = 0; y < kRows; y++) {
    for (size_t x = 0; x < kCols; x++) {
      auto point = points[k++];
      // Check point coordinates.
      EXPECT_NEAR(point.position.x, x, 0.000001f);
      EXPECT_NEAR(point.position.y, y, 0.000001f);
      EXPECT_NEAR(point.position.z, 1.01f, 0.000001f);
      // Check colour values.
      EXPECT_NEAR(point.color.x, 1.0f, 0.000001f);
      EXPECT_NEAR(point.color.y, 1.0f, 0.000001f);
      EXPECT_NEAR(point.color.z, 1.0f, 0.000001f);
    }
  }
}
