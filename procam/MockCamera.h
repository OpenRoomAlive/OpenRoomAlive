// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include "procam/BGRDCamera.h"
#include "procam/Display.h"


namespace dv { namespace procam {

/**
 * Mock implementation to run the application without an actual kinect.
 */
class MockCamera : public BGRDCamera {
 public:
  MockCamera(const std::shared_ptr<Display> display);

  cv::Mat getColorImage() override;
  cv::Mat getDepthImage() override;
  cv::Mat getUndistortedColorImage() override;
  CameraParams getParameters() override;
  void freshFrame() override {}
  cv::Mat undistort(const cv::Mat &HDImage, const cv::Mat &depthImage) override;
 private:
  std::shared_ptr<Display> display_;
};

}}
