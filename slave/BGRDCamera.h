// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <opencv2/opencv.hpp>

#include "core/ProCam.h"


namespace dv { namespace slave {


constexpr size_t kColorImageWidth = 1920;
constexpr size_t kColorImageHeight = 1080;
constexpr size_t kDepthImageWidth = 512;
constexpr size_t kDepthImageHeight = 424;
constexpr auto kColorFormat = CV_8UC4;
constexpr auto kDepthFormat = CV_32FC1;


/**
 * Interface to cameras capable of capturing color and depth.
 */
class BGRDCamera {
 public:
  virtual ~BGRDCamera();

  /**
   * Returns the last color frame from the camera.
   */
  virtual cv::Mat getColorImage() = 0;

  /**
   * Returns the last depth frame from the camera.
   */
  virtual cv::Mat getDepthImage() = 0;

  /**
   * Returns the last color frame for depth data.
   */
  virtual cv::Mat getUndistortedColorImage() = 0;

  /**
   * Returns the camera calibration parameters.
   */
  virtual CameraParams getParameters() = 0;

  /**
   * Waits for a fresh frame from the camera.
   */
  virtual void freshFrame() = 0;
};

}}

