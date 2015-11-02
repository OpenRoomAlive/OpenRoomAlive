// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <glm/glm.hpp>
#include <opencv2/opencv.hpp>

#include "core/ProCam.h"


namespace dv { namespace master {


/**
 * Class representing an ensemble of a Kinect and a Projector.
 */
class ProCam {
friend class Calibrator;

 public:
  ProCam() {}
  ProCam(
      const cv::Mat &colorCamMat,
      const cv::Mat &irCamMat,
      const cv::Mat &irDist,
      const dv::DisplayParams &displayParams);
  ~ProCam();

  /**
   * Returns the parameters of the display.
   */
  dv::DisplayParams getDisplayParams() const { return displayParams_; }

 private:
  /// Color camera matrix.
  const cv::Mat colorCamMat_;
  /// Ir camera matrix.
  const cv::Mat irCamMat_;
  /// Ir camera distortion coefficients.
  const cv::Mat irDist_;
  /// Parameters of the display.
  const dv::DisplayParams displayParams_;
  /// Color baseline.
  cv::Mat colorBaseline_;
  /// Depth baseline.
  cv::Mat depthBaseline_;
};

}}

