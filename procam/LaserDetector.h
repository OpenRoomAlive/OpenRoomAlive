// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include "core/Master.h"
#include "procam/BGRDCamera.h"
#include "procam/Display.h"


namespace dv { namespace procam {

/**
 * Class encapsulating laser detection in Kinect view.
 */
class LaserDetector {
 public:
  LaserDetector(
      const std::shared_ptr<Display>& display,
      const std::shared_ptr<MasterClient>& master,
      const std::shared_ptr<BGRDCamera>& camera,
      const std::shared_ptr<BaselineCapture>& baseline);

  /**
   * Detect lasers in Kinect view until display is turned off.
   */
  void detect();

 private:
  /// Display - OpenGL window.
  const std::shared_ptr<Display> display_;
  /// Master client handler.
  const std::shared_ptr<MasterClient> master_;
  /// Kinect camera implementation.
  const std::shared_ptr<BGRDCamera> camera_;
  /// Baseline capture.
  const std::shared_ptr<BaselineCapture> baseline_;
};

}}