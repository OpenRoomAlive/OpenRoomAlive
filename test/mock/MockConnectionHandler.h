// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <vector>
#include <unordered_map>

#include <boost/filesystem.hpp>

#include "master/ConnectionHandler.h"
#include "master/ProCamRecorder.h"

namespace dv { namespace master {

class MockConnectionHandler : public ConnectionHandler {
 public:
  /**
   * Creates a mock connection handler which uses the recorded images on
   * different procams to serve frames on request.
   */
  explicit MockConnectionHandler(const boost::filesystem::path &path);

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
  ParamMap getParams() override;

  /**
   * Retrieves pre-recorded color images from all procams.
   */
  FrameMap getGrayscaleImages() override;

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

  /**
   * Returns a pre-recorded set of depth variances.
   */
  FrameMap getDepthVariances() override;

  /**
   * Asks ProCams to start laser detection.
   */
  void startLaserDetection() override;

  /**
   * Asks ProCam to draw the path of a laser with given color.
   */
  void updateLaser(
      ConnectionID id,
      const std::vector<std::pair<cv::Point2i, cv::Point2i>> &path,
      const cv::Scalar &color) override;

 private:
  /**
   * Returns the number of files and subdirectories of the path directory.
   * Theprecondition is that the directory addressed by path contains only
   * subdirectories of with the data recorded for each proCam.
   */
  size_t countDirs(const boost::filesystem::path path);

  /**
   * Loads a frame with a specified Id from a given directory in OpenCV format.
   */
  cv::Mat loadFrame(ConnectionID id, ProCamRecorder::RecordedData dataType);

  /**
   * Loads frames for a given dataType from all procams.
   */
  FrameMap loadFrames(ProCamRecorder::RecordedData dataType);

  /**
   * Loads the camera parameters from a JSON file.
   */
  ProCamParam loadParam(ConnectionID id);

 private:
  /// Path to the directory storing images saved for each procam.
  const boost::filesystem::path path_;
  /// Number of connected proCams.
  size_t count_;
  /// Indexes of the next frames to be returned by a specified procam.
  std::vector<std::unordered_map<
      ProCamRecorder::RecordedData,
      size_t,
      ProCamRecorder::DataHasher>> nextFrame_;
};

}}
