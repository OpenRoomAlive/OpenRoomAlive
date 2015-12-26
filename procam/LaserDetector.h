// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <opencv2/opencv.hpp>

#include "core/Master.h"


namespace dv { namespace procam {

class BGRDCamera;
class BaselineCapture;

/**
 * Class encapsulating laser detection in Kinect view.
 */
class LaserDetector {
 public:
  LaserDetector(
      const std::shared_ptr<MasterClient>& master,
      const std::shared_ptr<BGRDCamera>& camera,
      const std::shared_ptr<BaselineCapture>& baseline);

  /**
   * Detect lasers in Kinect view until display is turned off.
   */
  void detect(
      const cv::Mat &frame,
      const cv::Mat &depth);

 private:
  using Candidates = std::vector<std::pair<cv::Point2i, int32_t>>;

  /// Master client handler.
  const std::shared_ptr<MasterClient> master_;
  /// Kinect camera implementation.
  const std::shared_ptr<BGRDCamera> camera_;
  /// Baseline capture.
  const std::shared_ptr<BaselineCapture> baseline_;

  /// Previous frame.
  cv::Mat prev_;
  /// True if a pointer is being tracked.
  bool tracked_;
  /// Point being tracked.
  cv::Point2i track_;
  /// Candidate points suitable for tracking.
  Candidates candidates_;
  /// Scaled down frame.
  cv::Mat scaled_;
  /// Temporary frame.
  cv::Mat frame_;
  /// Frame difference.
  cv::Mat diff_;
};

}}
