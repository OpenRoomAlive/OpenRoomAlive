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
  ProCam(ProCamID id, size_t frameWidth, size_t frameHeight);
  ~ProCam();

  /// Returns the ID of the camera.
  ProCamID getID() const { return id_; }

  // Disallow copy and assign.
  ProCam(const ProCam &) = delete;
  ProCam(ProCam &&) = delete;
  void operator = (const ProCam&) = delete;
  void operator = (ProCam &&) = delete;

  // Get width of the projector frame.
  size_t getFrameWidth();

  // Get height of the projector frame.
  size_t getFrameHeight();

 private:
  // ID of the camera.
  const ProCamID id_;
  // Width of the frame displayed by the projetor.
  const size_t frameWidth_;
  // Height of the frame displayed by the projector.
  const size_t frameHeight_;
};

} }
