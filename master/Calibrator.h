// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <set>
#include <vector>

#include <boost/functional/hash.hpp>

#include "master/ConnectionHandler.h"
#include "master/ProCamSystem.h"

namespace dv { namespace master {

/**
   * Class wrapping the functionality responsible for ProCam calibration.
 */
class Calibrator {
 private:
  /**
   * Hash for cv::Point2i.
   */
  struct cvPoint2iHasher {
    size_t operator() (const cv::Point2i &p) const {
      size_t seed = 0;
      boost::hash_combine(seed, p.x);
      boost::hash_combine(seed, p.y);
      return seed;
    }
  };

 public:
  /// Pair of procams that can see each other.
  using ProCamPair = std::pair<ConnectionID, ConnectionID>;

  /// Sequence of captured images from which gray codes are decoded.
  using CapturedMap = std::unordered_map<
      ProCamPair,
      std::vector<cv::Mat>,
      boost::hash<ProCamPair>>;

  /// Decoded gray code bit masks between a procam and a bgrd camera.
  using GrayCodeBitMaskMap = std::unordered_map<
      ProCamPair,
      cv::Mat,
      boost::hash<ProCamPair>>;

  using ColorProjectorMap = std::unordered_map<
      ProCamPair,
      std::unordered_map<cv::Point2i, cv::Point2i, cvPoint2iHasher>,
      boost::hash<ProCamPair>>;

  Calibrator(
      const std::vector<ConnectionID>& ids,
      const boost::shared_ptr<ConnectionHandler>& connectionHandler,
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
   * Decodes the gray codes.
   */
  void decodeGrayCodes();

  /**
   * Calibrates the ProCam system.
   */
  void calibrate();

 private:
  /**
   * Decode the captured gray code pattern into a bit mask.
   */
  GrayCodeBitMaskMap decodeToBitMask();

  /**
   * Constructs the mapping from points in the color image to
   * the points in the projector screen space.
   */
  void colorToProjPoints(GrayCodeBitMaskMap &decodedGrayCodes);

  /*
   * Filters noise from decoded gray code.
   */
  void removeNoise(const cv::Mat &grayCode, ConnectionID id);

 private:
  /// IDs of the procam connections
  const std::vector<ConnectionID> ids_;
  /// Pointer to the connection handler
  const boost::shared_ptr<ConnectionHandler> connectionHandler_;
  /// ProCam system.
  const std::shared_ptr<ProCamSystem> system_;
  /// Map of captured gray code pattern
  CapturedMap captured_;
  /**
   * Contains mappings from the points in the kinect color image
   * to the points in the projector's screen.
   */
  ColorProjectorMap colorToProj_;

};

}}

