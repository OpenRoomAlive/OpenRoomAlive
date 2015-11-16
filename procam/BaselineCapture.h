// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <mutex>

#include <folly/Optional.h>

#include <opencv2/opencv.hpp>


namespace dv { namespace procam {

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

  /**
   * Returns the depth variance.
   */
  cv::Mat getDepthVariance();

  /**
   * Blocks until the baseline and variance are computed.
   */
  void framesProcessed();

 private:
  /// Number of candidate frames to use to find the still baseline. Pre: > 1.
  static constexpr int kCandidateFrames = 100;
  /// Buffer containing candidate frames.
  std::array<cv::Mat, kCandidateFrames> frames_;
  /// Number of frames processed.
  uint64_t count_;
  /// Mutex protecting the frame count.
  std::mutex countLock_;
  /// Condition variable waiting for the baseline and variance to be computed.
  std::condition_variable countCond_;
  /// Extracted depth baseline.
  cv::Mat baseline_;
  /// Extracted depth variance.
  cv::Mat variance_;
};

}}
