// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <chrono>

#include <glm/glm.hpp>
#include <opencv2/opencv.hpp>

#include "core/Types.h"

namespace dv { namespace master {

class Calibrator;
class ProCamSystem;

/**
 * Class representing an ensemble of a Kinect and a Projector.
 */
class ProCam {
  friend class Calibrator;
  friend class ProCamSystem;

 public:
  ProCam();
  ProCam(
      const cv::Mat &colorCamMat,
      const cv::Mat &irCamMat,
      const cv::Mat &irDist,
      const cv::Size &actualProjRes,
      const cv::Size &effectiveProjRes,
      const std::chrono::milliseconds &latency);
  ~ProCam();

  /**
   * Returns the actual resolution.
   */
  cv::Size getActualResolution() const { return actualProjRes_; }

  /**
   * Returns the latency.
   */
  std::chrono::milliseconds getLatency() const { return latency_; }

 private:
  /// Color camera matrix.
  const cv::Mat colorCamMat_;
  /// Ir camera matrix.
  const cv::Mat irCamMat_;
  /// Ir camera distortion coefficients.
  const cv::Mat irDist_;
  /// Parameters of the display.
  const cv::Size actualProjRes_;
  /// Effective parameters of the display.
  const cv::Size effectiveProjRes_;
  /// Wait time between displaying and reading an image.
  const std::chrono::milliseconds latency_;
  /// Projector matrix.
  cv::Mat projMat_;
  /// Projector distortion coefficients.
  cv::Mat projDist_;
  /// Color baseline.
  cv::Mat colorBaseline_;
  /// Depth baseline.
  cv::Mat depthBaseline_;
  /// Variance depth image.
  cv::Mat depthVariance_;
  /// Cameras that can observe some part of projector's image.
  std::vector<ConnectionID> projectorGroup_;
};

}}

