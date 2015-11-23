// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include "procam/MockCamera.h"

using namespace dv;
using namespace dv::procam;

MockCamera::MockCamera(uint16_t logLevel, const std::string &logFilename) {
  std::cout
    << "Logging to \"" << logFilename << "\" at level " << logLevel << "."
    << std::endl;
}

cv::Mat MockCamera::getColorImage() {
  // 1x1 RGB image.
  return cv::Mat(
      kColorImageWidth,
      kColorImageHeight,
      kColorFormat,
      cv::Scalar(0, 0, 0));
}

cv::Mat MockCamera::getDepthImage() {
  // 1x1 32 bit float depth.
  return cv::Mat(
      kDepthImageWidth,
      kDepthImageHeight,
      kDepthFormat,
      cv::Scalar(0.0f));
}

cv::Mat MockCamera::getUndistortedColorImage() {
  return cv::Mat(
      kDepthImageWidth,
      kDepthImageHeight,
      kColorFormat,
      cv::Scalar(0, 0, 0));
}

CameraParams MockCamera::getParameters() {
  return CameraParams();
}

cv::Mat MockCamera::undistort(
    const cv::Mat &HDImage,
    const cv::Mat &depthImage)
{
  (void) HDImage;
  (void) depthImage;
  return cv::Mat(kColorImageHeight, kColorImageWidth, kColorFormat);
}
