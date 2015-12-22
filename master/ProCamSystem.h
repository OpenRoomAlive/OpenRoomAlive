// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <cstdint>

#include <atomic>
#include <memory>
#include <mutex>
#include <unordered_map>

#include <folly/dynamic.h>

#include "master/MasterConnectionHandler.h"
#include "master/ProCam.h"

namespace dv { namespace master {

class Calibrator;
class ProCam;

/**
 * Class managing an ensemble of ProCam units.
 */
class ProCamSystem {
friend class Calibrator;
friend class LaserDrawer;

 public:
  ProCamSystem();
  ~ProCamSystem();

  /**
   * Loads the system from a json object.
   */
  void fromJSON(const folly::dynamic &data);

  /**
   * Saves the system to a file.
   */
  folly::dynamic toJSON() const;

  /**
   * Adds a new ProCam to the system.
   */
  void addProCam(
      ConnectionID id,
      const CameraModel &colorCam,
      const CameraModel &irCam,
      const cv::Size &actualProjRes,
      const cv::Size &effectiveProjRes,
      const std::chrono::milliseconds &latency);

  /**
   * Returns pointer to ProCam if there exists one with given id.
   */
  std::shared_ptr<const ProCam> getProCam(ConnectionID id) const;

  // Disallow copy and assign.
  ProCamSystem(const ProCamSystem &) = delete;
  ProCamSystem(ProCamSystem &&) = delete;
  void operator = (const ProCamSystem&) = delete;
  void operator = (ProCamSystem &&) = delete;

 private:
  /**
   * Non const ProCam getter.
   */
  std::shared_ptr<ProCam> getProCam(ConnectionID id);

 private:
  /// Lock protecting access to the procam system.
  std::mutex lock_;
  /// Hash map storing all procams.
  std::unordered_map<ConnectionID, std::shared_ptr<ProCam>> proCams_;
};

}}

