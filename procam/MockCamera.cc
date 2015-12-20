// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include "core/GrayCode.h"
#include "procam/MockCamera.h"

using namespace dv;
using namespace dv::procam;


MockCamera::MockCamera(const std::shared_ptr<Display> display)
    : display_(display)
{
}

cv::Mat MockCamera::getColorImage() {
  // 24 bit full HD image.
  cv::Mat fullImage;
  cv::resize(display_->getImage(), fullImage, cv::Size(kColorImageWidth, kColorImageHeight));
  cv::cvtColor(fullImage, fullImage, CV_GRAY2BGRA);
  return fullImage;
}

cv::Mat MockCamera::getDepthImage() {
  // 32 bit float depth image.
  return cv::Mat(
      kDepthImageWidth,
      kDepthImageHeight,
      kDepthFormat,
      cv::Scalar(1.0f));
}

cv::Mat MockCamera::getUndistortedColorImage() {
  // 24 bit image.
  cv::Mat undistorted;
  cv::resize(display_->getImage(), undistorted, cv::Size(kDepthImageHeight, kDepthImageWidth));
  return undistorted;
}

CameraParams MockCamera::getParameters() {
  CameraParams cameraParams;

  // Retrieve the color camera params from the device.
  {
    cameraParams.bgr.fx = 1.0f;
    cameraParams.bgr.fy = 1.0f;
    cameraParams.bgr.cx = 1.0f;
    cameraParams.bgr.cy = 1.0f;

    cameraParams.bgr.shift_d = 0;
    cameraParams.bgr.shift_m = 0;

    cameraParams.bgr.mx_x3y0 = 0;
    cameraParams.bgr.mx_x0y3 = 0;
    cameraParams.bgr.mx_x2y1 = 0;
    cameraParams.bgr.mx_x1y2 = 0;
    cameraParams.bgr.mx_x2y0 = 0;
    cameraParams.bgr.mx_x0y2 = 0;
    cameraParams.bgr.mx_x1y1 = 0;
    cameraParams.bgr.mx_x1y0 = 0;
    cameraParams.bgr.mx_x0y1 = 0;
    cameraParams.bgr.mx_x0y0 = 0;
    cameraParams.bgr.my_x3y0 = 0;
    cameraParams.bgr.my_x0y3 = 0;
    cameraParams.bgr.my_x2y1 = 0;
    cameraParams.bgr.my_x1y2 = 0;
    cameraParams.bgr.my_x2y0 = 0;
    cameraParams.bgr.my_x0y2 = 0;
    cameraParams.bgr.my_x1y1 = 0;
    cameraParams.bgr.my_x1y0 = 0;
    cameraParams.bgr.my_x0y1 = 0;
    cameraParams.bgr.my_x0y0 = 0;
  }

  // Retrieve the ir camera params from the device.
  {
    cameraParams.ir.fx = 1.0f;
    cameraParams.ir.fy = 1.0f;
    cameraParams.ir.cx = 1.0f;
    cameraParams.ir.cy = 1.0f;

    cameraParams.ir.k1 = 0;
    cameraParams.ir.k2 = 0;
    cameraParams.ir.k3 = 0;
    cameraParams.ir.p1 = 0;
    cameraParams.ir.p2 = 0;
  }

  return cameraParams;
}

cv::Mat MockCamera::undistort(
    const cv::Mat &HDImage,
    const cv::Mat &depthImage)
{
  // Hardcoded to CV_32S because we are undistorting graycode
  return cv::Mat(depthImage.size(), CV_32S, HDImage.data);
}
