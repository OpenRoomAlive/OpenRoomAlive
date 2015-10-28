// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <cstdint>

#include <atomic>
#include <memory>
#include <mutex>
#include <unordered_map>

#include "master/MasterConnectionHandler.h"
#include "master/ProCam.h"

namespace dv { namespace master {

class ProCam;

/**
 * Class managing an ensemble of ProCam units.
 */
class ProCamSystem {
 public:
  ProCamSystem();
  ~ProCamSystem();

  /**
   * Adds a new camera to the system.
   */
  void addCamera(
      ConnectionID id,
      const dv::CameraParams &camParams,
      const dv::DisplayParams &displayParams);

  /**
   * Returns the parameters of the Color and Ir cameras of given ProCam.
   */
  dv::CameraParams getCameraParams(ConnectionID id);

  /**
   * Returns the parameters of the display of given ProCam.
   */
  dv::DisplayParams getDisplayParams(ConnectionID id);

  // Disallow copy and assign.
  ProCamSystem(const ProCamSystem &) = delete;
  ProCamSystem(ProCamSystem &&) = delete;
  void operator = (const ProCamSystem&) = delete;
  void operator = (ProCamSystem &&) = delete;

 private:
  /// Lock protecting access to the procam system.
  std::mutex lock_;
  /// Hash map storing all procams.
  std::unordered_map<ConnectionID, ProCam> proCams_;
};

} }
