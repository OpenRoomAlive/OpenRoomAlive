// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include "procam/BGRDCamera.h"


namespace dv { namespace procam {

/**
 * Mock implementation to run the application without an actual kinect.
 */
class MockCamera : public BGRDCamera {
 public:
  MockCamera(uint16_t logLevel, const std::string &logFilename);

  cv::Mat getColorImage() override;
  cv::Mat getDepthImage() override;
  cv::Mat getUndistortedColorImage() override;
  CameraParams getParameters() override;
  void freshFrame() override {}
};

}}

