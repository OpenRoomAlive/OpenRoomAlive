// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <chrono>

#include <opencv2/opencv.hpp>

#include "core/Master.h"


namespace dv { namespace procam {

class Display;
class BGRDCamera;
class BaselineCapture;


/**
 * Class encapsulating laser detection in Kinect view.
 */
class LaserDetector {
 public:
  LaserDetector(
      const std::shared_ptr<Display>& display,
      const std::shared_ptr<MasterClient>& master,
      const std::shared_ptr<BGRDCamera> &camera,
      const std::shared_ptr<BaselineCapture>& baseline);

  /**
   * Detect lasers in Kinect view until display is turned off.
   */
  void detect(
      const cv::Mat &rgb,
      const cv::Mat &depth);

 private:
  /// Single candidate point.
  struct Candidate {
    /// Coordinate of the candidate in depth image space.
    cv::Point2i point;
    /// Age of the candidate.
    int32_t age;
    /// Last frame when it was touched.
    uint64_t last;
  };

  /// List of candidate points with age.
  using Candidates = std::vector<Candidate>;

  /// Structure describing a single laser pointer.
  struct Pointer {
    /// Position of the pointer.
    cv::Point2i position;
    /// Last time this pointer was tracked.
    uint64_t frame;

    /**
     * Initializes an empty pointer.
     */
    Pointer()
      : position(-1, -1)
      , frame(0)
    {
    }

    /**
     * Creates a new pointer.
     */
    Pointer(const cv::Point2i &_position, uint64_t _frame)
      : position(_position)
      , frame(_frame)
    {
    }
  };

 private:
  /// Tracks the laser in the whole image.
  void acquirePointers();
  /// Tracks the laser in a window around the old coordinate.
  void trackPointer(Pointer &pointer);

  /// Display - OpenGL window.
  const std::shared_ptr<Display> display_;
  /// Master client handler.
  const std::shared_ptr<MasterClient> master_;
  /// Kinect camera implementation.
  const std::shared_ptr<BGRDCamera> camera_;
  /// Baseline capture.
  const std::shared_ptr<BaselineCapture> baseline_;

  /// Previous image.
  cv::Mat prev_;
  /// Current frame.
  cv::Mat frame_;
  /// Current depth.
  cv::Mat depth_;
  /// Difference between two rgb frames.
  cv::Mat diff_;
  /// Mask based on baseline & variance.
  cv::Mat mask_;
  /// Possible laser location map.
  cv::Mat laser_;

  /// Frame counter.
  uint64_t counter_;
  /// List of candidate points used to acquire new pointers.
  Candidates candidates_;

  cv::Mat temp;

  /// List of tracked laser pointer.
  std::vector<Pointer> pointers;
};

}}
