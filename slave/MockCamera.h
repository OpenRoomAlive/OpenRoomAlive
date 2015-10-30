// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include "slave/RGBDCamera.h"


namespace dv { namespace slave {

/**
 * Mock implementation to run the application without an actual kinect.
 */
class MockCamera : public RGBDCamera {
 public:
  cv::Mat getRGBImage() override;
  cv::Mat getDepthImage() override;
  cv::Mat getUndistortedRGBImage() override;
  CameraParams getParameters() override;
  void warmup() override {}
};

}}

