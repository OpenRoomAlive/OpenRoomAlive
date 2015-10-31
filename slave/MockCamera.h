// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include "slave/BGRDCamera.h"


namespace dv { namespace slave {

/**
 * Mock implementation to run the application without an actual kinect.
 */
class MockCamera : public BGRDCamera {
 public:
  cv::Mat getColorImage() override;
  cv::Mat getDepthImage() override;
  cv::Mat getUndistortedColorImage() override;
  CameraParams getParameters() override;
  void warmup() override {}
};

}}

