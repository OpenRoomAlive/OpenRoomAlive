// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <opencv2/opencv.hpp>

#include "core/Types.h"

namespace dv { namespace master {

/**
 * Class representing an event associated with laser detection by a ProCam.
 */
class Event {
 public:

  Event(const ConnectionID id, const cv::Point3f point, const cv::Scalar color);

  ~Event();

  cv::Point3f getPosition() const { return point_; };

  ConnectionID getProCamID() const { return id_; };

  cv::Scalar getColor() const { return color_; };

 private:
  /// ID of the ProCam that sent the event.
  const ConnectionID id_;
  /// Point representing new detected position of the laser.
  const cv::Point3f point_;
  /// Color of the detected laser.
  const cv::Scalar color_;
};

} }
