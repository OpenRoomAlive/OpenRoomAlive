// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <glm/glm.hpp>

#include "core/ProCam.h"

namespace dv { namespace master {


/**
 * Class representing an ensemble of a Kinect and a Projector.
 */
class ProCam {
 public:
  ProCam() {}
  ProCam(
      const dv::CameraParams &camParams,
      const dv::DisplayParams &displayParams);
  ~ProCam();


  /**
   * Returns the parameters of the Color and Ir cameras.
   */
  dv::CameraParams getCameraParams() const { return cameraParams_; }

  /**
   * Returns the parameters of the display.
   */
  dv::DisplayParams getDisplayParams() const { return displayParams_; }


 private:
  /// Parameters of the Color and Ir cameras.
  const dv::CameraParams cameraParams_;
  /// Parameters of the display.
  const dv::DisplayParams displayParams_;
};

} }
