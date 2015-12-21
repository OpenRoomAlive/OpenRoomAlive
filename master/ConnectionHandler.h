// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <unordered_map>
#include <vector>

#include <opencv2/opencv.hpp>

#include "core/Master.h"
#include "core/Types.h"

namespace dv { namespace master {

class ConnectionHandler {
 public:
  /// Map from IDs of connections to captured images.
  using FrameMap = std::unordered_map<ConnectionID, cv::Mat>;
  /// Map from IDs of connections to parameters.
  using ParamMap = std::unordered_map<ConnectionID, ProCamParam>;

  virtual ~ConnectionHandler();

  /**
   * Blocks until a specific number of clients connect.
   * Returns the IDs of the clients that connected.
   */
  virtual std::vector<ConnectionID> waitForConnections(size_t count) = 0;

  /**
   * Sends a signal to a given client to display specified gray code.
   */
  virtual void displayGrayCode(
      ConnectionID id,
      Orientation::type orientation,
      int16_t level,
      bool invertedGrayCode) = 0;

  /**
   * Sends signal to a given client to display white image.
   */
  virtual void displayWhite(ConnectionID id) = 0;

  /**
   * Clears the display of a client.
   */
  virtual void clearDisplay(ConnectionID id) = 0;

  /**
   * Disconnects all clients.
   */
  virtual void stop() = 0;

  /**
   * Clears the displays of all clients.
   */
  virtual void clearDisplays() = 0;

  /**
   * Invokes getParams on all clients.
   */
  virtual ParamMap getParams() = 0;

  /**
   * Invokes getColorImage on all clients.
   */
  virtual FrameMap getGrayscaleImages() = 0;

  /**
   * Retrieves the depth image of a client.
   */
  virtual cv::Mat getDepthImage(ConnectionID id) = 0;

  /**
   * Invokes getDepthImage on all clients.
   */
  virtual FrameMap getDepthImages() = 0;

  /**
   * Invokes getUndistortedColorImage on all clients.
   */
  virtual FrameMap getUndistortedColorImages() = 0;

  /**
   * Invokes getColorBaseline on all clients.
   */
  virtual FrameMap getColorBaselines() = 0;

  /**
   * Invokes getDepthBaseline on all clients.
   */
  virtual FrameMap getDepthBaselines() = 0;

  /**
   * Invokes getDepthVariance on all clients.
   */
  virtual FrameMap getDepthVariances() = 0;

  /**
   * Asks porcam to undistort the provided HD image.
   */
  virtual cv::Mat undistort(ConnectionID id, const cv::Mat &imageHD) = 0;

  /**
   * Asks ProCams to start detecting the laser pointer.
   */
  virtual void startLaserDetection() = 0;

  /**
   * Asks ProCam to draw the path of a laser with given color.
   */
  virtual void updateLaser(
      ConnectionID id,
      const std::vector<std::pair<cv::Point2i, cv::Point2i>> &path,
      const cv::Scalar &color) = 0;
};

}}
