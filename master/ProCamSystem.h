// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <cstdint>

#include <atomic>
#include <memory>
#include <mutex>
#include <unordered_map>

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
  std::shared_ptr<ProCam> addCamera();

  // Disallow copy and assign.
  ProCamSystem(const ProCamSystem &) = delete;
  ProCamSystem(ProCamSystem &&) = delete;
  void operator = (const ProCamSystem&) = delete;
  void operator = (ProCamSystem &&) = delete;

 private:
  /// Lock protecting access to the procam system.
  std::mutex lock_;
  /// Counter providing IDs for procams.
  std::atomic<ProCamID> nextID_;
  /// Hash map storing all procams.
  std::unordered_map<ProCamID, std::shared_ptr<ProCam>> proCams_;
};

} }
