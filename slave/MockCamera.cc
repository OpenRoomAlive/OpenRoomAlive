// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include "slave/MockCamera.h"

using namespace dv;
using namespace dv::slave;

MockCamera::MockCamera(uint16_t logLevel, const std::string &logFilename) {
  std::cout 
    << "Logging to \"" << logFilename << "\" at level " << logLevel << "."
    << std::endl;
}

cv::Mat MockCamera::getColorImage() {
  // 1x1 RGB image.
  return cv::Mat(1, 1, kColorFormat);
}

cv::Mat MockCamera::getDepthImage() {
  // 1x1 32 bit float depth.
  return cv::Mat(1, 1, kDepthFormat);
}

cv::Mat MockCamera::getUndistortedColorImage() {
  return cv::Mat(1, 1, kColorFormat);
}

CameraParams MockCamera::getParameters() {
  return CameraParams();
}

