// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <opencv2/opencv.hpp>

#include "core/ProCam.h"


namespace dv { namespace slave {

/**
 * Interface to RGBD cameras.
 */
class RGBDCamera {
 public:
  virtual ~RGBDCamera();

  /**
   * Returns the last RGB frame from the camera.
   */
  virtual cv::Mat getRGBImage() = 0;

  /**
   * Returns the last depth frame from the camera.
   */
  virtual cv::Mat getDepthImage() = 0;

  /**
   * Returns the last RGB frame for depth data.
   */
  virtual cv::Mat getUndistortedRGBImage() = 0;

  /**
   * Returns the camera calibration parameters.
   */
  virtual CameraParams getParameters() = 0;
};

}}

