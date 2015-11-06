// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <set>
#include <vector>

#include <boost/functional/hash.hpp>

#include "master/MasterConnectionHandler.h"
#include "master/ProCamSystem.h"

namespace dv { namespace master {

/**
   * Class wrapping the functionality responsible for ProCam calibration.
 */
class Calibrator {
 public:
  /// Pair of procams that can see each other.
  using ProCamPair = std::pair<ConnectionID, ConnectionID>;

  /// Sequence of captured images from which gray codes are decoded.
  using CapturedMap = std::unordered_map<
      ProCamPair,
      std::vector<cv::Mat>,
      boost::hash<ProCamPair>>;

 public:
  /// Decoded gray codes between a procam and a bgrd camera.
  using GrayCodeMap = std::unordered_map<
      ProCamPair,
      cv::Mat,
      boost::hash<ProCamPair>>;
  using CapturedPixelsMap = std::unordered_map<
      std::pair<ConnectionID, ConnectionID>,
      std::set<std::pair<size_t, size_t>>,
      boost::hash<ProCamPair>>;

  Calibrator(
      const std::vector<ConnectionID>& ids,
      const boost::shared_ptr<MasterConnectionHandler>& connectionHandler,
      const std::shared_ptr<ProCamSystem>& system);
  ~Calibrator();

  /**
   * Captures and saves baselines.
   */
  void captureBaselines();

  /**
   * Forms projector groups.
   */
  void formProjectorGroups();

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
   * Decode the captured gray code pattern into a bit mask.
   */
  GrayCodeMap decode();

  /**
   * Retrieve pixel coordinates from the captured gray codes.
   */
  static CapturedPixelsMap grayCodesToPixels(GrayCodeMap &decodedGrayCodes);

  /**
   * Converts gray code encoding to binary according to the method outlined
   * at: http://www.electrical4u.com.
   */
  static inline uint32_t grayCodeToBinary(
      uint32_t encodedBits,
      size_t signifBits)
  {
    // Set the MSB of the binary number to MSB of the gray code encoding.
    uint32_t res = encodedBits & (1 << (signifBits - 1));

    bool prevSet = res > 0;

    for (int i = signifBits - 2; i >= 0; i--) {
      // If the current bit of the encoding is 1, flip the preSet flag.
      if (encodedBits & (1 << i)) {
        prevSet = !prevSet;
      }
      // If prevSet flag is set, set the current bit of the binary result.
      res = prevSet ? res | (1 << i) : res;
    }
    return res;
  }

 private:
  /// IDs of the procam connections
  const std::vector<ConnectionID> ids_;
  /// Pointer to the connection handler
  const boost::shared_ptr<MasterConnectionHandler> connectionHandler_;
  /// ProCam system.
  const std::shared_ptr<ProCamSystem> system_;
  /// Map of captured gray code pattern
  CapturedMap captured_;
};

} }
