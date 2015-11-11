// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <array>

#include <folly/Optional.h>

#include <opencv2/opencv.hpp>


namespace dv { namespace slave {

/**
 * Class that is responsible for capture the depth baseline.
 */
class BaselineCapture {
 public:
  BaselineCapture();
  ~BaselineCapture();

  /**
   * Processes a new frame.
   */
  void process(const cv::Mat &frame);

  /**
   * Returns the baseline depth image.
   */
  cv::Mat getDepthImage();

 private:
  /// Number of candidate frames to use to find the still baseline.
  static constexpr int kCandidateFrames = 10;
  /// Buffer containing candidate frames.
  std::array<cv::Mat, kCandidateFrames> frames_;
  /// Number of frames processed.
  uint64_t count_;
  /// Extracted depth baseline.
  cv::Mat depth_;
  /// Previous frame.
  cv::Mat prev_;
};

}}
