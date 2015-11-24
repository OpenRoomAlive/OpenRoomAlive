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

  /**
   * Returns a pre-recorded set of depth variances.
   */
  FrameMap getDepthVariances() override;

  /**
   * Undistorts an image. Not sure how.
   */
  cv::Mat undistort(ConnectionID id, const cv::Mat &imageHD) override;

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
   * Loads the camera parameters from an XML file.
   */
  CameraParams loadCameraParams(ConnectionID id);

  /**
   * Loads display parameters from a text file.
   */
  DisplayParams loadDisplayParams(ConnectionID id);

  /**
   * Given the type of the frame that the user wishes to retrieve, returns
   * the name of the file that should be loaded into a cv::Mat.
   */
  std::string frameName(ConnectionID id, ProCamRecorder::RecordedData dataType);

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
