// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <chrono>
#include <unordered_map>

#include <glm/glm.hpp>
#include <opencv2/opencv.hpp>

#include <boost/functional/hash.hpp>

#include "core/Types.h"

namespace dv { namespace master {

class Calibrator;
class ProCamSystem;

/**
 * Class representing an ensemble of a Kinect and a Projector.
 */
class ProCam {
  friend class Calibrator;
  friend class LaserDrawer;
  friend class PointCloud;
  friend class ProCamSystem;

 public:
  ProCam();
  ProCam(
      const CameraModel &colorCam,
      const CameraModel &irCam,
      const cv::Size &actualProjRes,
      const cv::Size &effectiveProjRes,
      const std::chrono::milliseconds &latency);
  ~ProCam();

  cv::Size getActualResolution() const { return actualProjRes_; }
  cv::Size getEffectiveResolution() const { return effectiveProjRes_; }

  std::chrono::milliseconds getLatency() const { return latency_; }

  CameraModel getColorCameraModel() const { return colorCam_; }
  CameraModel getIrCameraModel() const { return irCam_; }

  cv::Mat getProjMatrix() const { return projMat_; }
  cv::Mat getProjDistortion() const { return projDist_; }

  cv::Mat getBaselineColor() const { return colorBaseline_; }
  cv::Mat getBaselineDepth() const { return depthBaseline_; }
  cv::Mat getBaselineVariance() const { return depthVariance_; }

  CameraPose getPose(const ConnectionID &id) const;

 private:
  /// Color camera matrix & distortion.
  const CameraModel colorCam_;
  /// Ir camera matrix & distortion.
  const CameraModel irCam_;
  /// Parameters of the display.
  const cv::Size actualProjRes_;
  /// Effective parameters of the display.
  const cv::Size effectiveProjRes_;
  /// Wait time between displaying and reading an image.
  const std::chrono::milliseconds latency_;
  /// Projector calibration matrix.
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
  /// Stores poses of the kinects in the projector group.
  std::unordered_map<ConnectionID, CameraPose, boost::hash<ConnectionID>> poses_;
};

}}

