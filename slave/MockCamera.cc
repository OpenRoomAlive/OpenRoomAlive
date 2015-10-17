// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include "slave/MockCamera.h"

using namespace dv;
using namespace dv::slave;


cv::Mat MockCamera::getRGBFrame() {
  // 1x1 RGB image.
  return cv::Mat(1, 1, CV_8UC3);
}

cv::Mat MockCamera::getDepthFrame() {
  // 1x1 16 bit depth.
  return cv::Mat(1, 1, CV_16UC3);
}

CameraParams MockCamera::getParameters() {
  return CameraParams();
}
