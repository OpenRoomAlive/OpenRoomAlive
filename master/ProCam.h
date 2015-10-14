// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <glm/glm.hpp>

namespace dv { namespace master {

/**
 * All procams will get numeric IDs, assigned sequentially.
 */
using ProCamID = uint64_t;


/**
 * Class representing an ensemble of a Kinect and a Projector.
 */
class ProCam {
 public:
  ProCam(ProCamID id);
  ~ProCam();

  /// Returns the ID of the camera.
  ProCamID getID() const { return id_; }

  // Disallow copy and assign.
  ProCam(const ProCam &) = delete;
  ProCam(ProCam &&) = delete;
  void operator = (const ProCam&) = delete;
  void operator = (ProCam &&) = delete;

 private:
  // ID of the camera.
  const ProCamID id_;
};

} }
