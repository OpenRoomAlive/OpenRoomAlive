// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <vector>
#include <unordered_map>

#include "master/ConnectionHandler.h"

namespace dv { namespace master {

class MockConnectionHandler : public ConnectionHandler {
 public:
  /**
   * Creates a mock connection handler which uses the recorded images on
   * different procams to serve frames on request.
   */
  explicit MockConnectionHandler(const std::string &path);

  /**
   * Returns the IDs of connected procams.
   */
  std::vector<ConnectionID> waitForConnections(size_t count) override;

  /**
   * Sends a signal to a given procam to display specified gray code.
   */
  void displayGrayCode(
      ConnectionID id,
      Orientation::type orientation,
      int16_t level,
      bool invertedGrayCode) override;

  /**
   * Sends signal to a given procam to display white image.
   */
  void displayWhite(ConnectionID id) override;

  /**
   * Clears the display of a procam.
   */
  void clearDisplay(ConnectionID id) override;

  /**
   * Disconnects all procams.
   */
  void stop() override;

  /**
   * Clears the displays of all procams.
   */
  void clearDisplays() override;

  /**
   * Retrieves camera params of all connected procams.
   */
  std::unordered_map<ConnectionID, CameraParams> getCamerasParams() override;

  /**
   * Retrieves display params of all connected procams.
   */
  std::unordered_map<ConnectionID, DisplayParams> getDisplaysParams() override;

  /**
   * Retrieves pre-recorded color images from all procams.
   */
  FrameMap getColorImages() override;

  /**
   * Retrieves a pre-recorded depth image from a specified procam.
   */
  cv::Mat getDepthImage(ConnectionID id) override;

  /**
   * Returns a pre-recorded set of depth images from all procam units.
   */
  FrameMap getDepthImages() override;

  /**
   * Returns a pre-recorded set of undistorted color images.
   */
  FrameMap getUndistortedColorImages() override;

  /**
   * Returns a pre-recorded set of color baselines.
   */
  FrameMap getColorBaselines() override;

  /**
   * Returns a pre-recorded set of depth baselines.
   */
  FrameMap getDepthBaselines() override;

 private:
  /// Path to the directory storing images saved for each procam.
  const std::string path_;
  /// IDs of the connected proCams.
  std::vector<ConnectionID> ids_;
  /// Indexes of the next frames to be returned by a specified procam.
  std::unordered_map<ConnectionID, std::vector<size_t>> nextFrame_;
};

}}
