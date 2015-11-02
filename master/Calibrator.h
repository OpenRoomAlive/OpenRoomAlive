// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <vector>

#include "master/MasterConnectionHandler.h"
#include "master/ProCamSystem.h"

namespace dv { namespace master {

/**
 * Class wrapping the functionality responsible for ProCam calibration.
 */
class Calibrator {
 private:
  /**
   * Class to hash pairs of connections.
   */
  struct ConnectionPairHasher {
    size_t operator() (const std::pair<ConnectionID, ConnectionID> &pair) const {
      return pair.first + (pair.second << 31);
    }
  };

 public:
  using GrayCodeMap = std::unordered_map<
      std::pair<ConnectionID, ConnectionID>,
      cv::Mat,
      ConnectionPairHasher>;

  Calibrator(
      const std::vector<ConnectionID>& ids,
      const boost::shared_ptr<MasterConnectionHandler>& connectionHandler,
      const std::shared_ptr<ProCamSystem>& system);
  ~Calibrator();

  /**
   * Function which loops over all connections (procams) and for each
   * procam it sends requests to display a sequence of gray codes. It
   * should be run in a separate thread.
   */
  void displayGrayCodes();

  /**
   * Display and capture a single level of gray code.
   */
  void displayAndCapture(
      ConnectionID id,
      Orientation::type orientation,
      size_t level,
      bool inverted);

  /**
   * Captures and saves baselines.
   */
  void captureBaselines();

  /**
   * Decode the captured gray code pattern into a bit mask.
   */
  GrayCodeMap decode();

 private:
  /// IDs of the procam connections
  const std::vector<ConnectionID> ids_;
  /// Pointer to the connection handler
  const boost::shared_ptr<MasterConnectionHandler> connectionHandler_;
  /// ProCam system.
  const std::shared_ptr<ProCamSystem> system_;
  /// Map of captured gray code pattern
  GrayCodeMap captured_;
};

} }
