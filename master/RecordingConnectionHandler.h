// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include "master/MasterConnectionHandler.h"
#include "master/ProCamRecorder.h"

namespace dv { namespace master {

class RecordingConnectionHandler : public MasterConnectionHandler {
 public:
  RecordingConnectionHandler(
      uint16_t proCamPort,
      const std::shared_ptr<EventStream>& streamconst,
      const std::string &recordDirectory);
  ~RecordingConnectionHandler();

  /**
   * Blocks until a specific number of procams connect. For every procam it
   * creates a directory where all data retrieved from the proCam will be
   * stored.
   * Returns the IDs of the ProCams that connected.
   */
  std::vector<ConnectionID> waitForConnections(size_t count) override;

  /**
   * Invokes getColorImage on all clients and records the retrieved frames.
   */
  FrameMap getColorImages() override;

  /**
   * Retrieves the depth image of a procam and records it.
   */
  cv::Mat getDepthImage(ConnectionID id) override;

  /**
   * Invokes getDepthImage on all clients and records the retrieved frames.
   */
  FrameMap getDepthImages() override;

  /**
   * Invokes getUndistortedColorImage on all clients and records the retrieved
   * frames.
   */
  FrameMap getUndistortedColorImages() override;

  /**
   * Invokes getColorBaseline on all clients and records the retrieved frames.
   */
  FrameMap getColorBaselines() override;

  /**
   * Invokes getDepthBaseline on all clients and records the retrieved frames.
   */
  FrameMap getDepthBaselines() override;

  /**
   * Invokes getDepthVariance on all clients and records the retrieved frames.
   */
  FrameMap getDepthVariances() override;

  /**
   * Asks porcam to undistort the provided HD image and records the result.
   */
  cv::Mat undistort(ConnectionID id, const cv::Mat &imageHD) override;

  /**
   * Invokes getCameraParams on all clients and records the retrieved camera
   * data.
   */
  std::unordered_map<ConnectionID, CameraParams> getCamerasParams() override;

  /**
   * Invokes getDisplayParams on all clients and records the retrieved display
   * data.
   */
  std::unordered_map<ConnectionID, DisplayParams> getDisplaysParams() override;

 private:
  void recordAll(
      const FrameMap &capturedFrames,
      ProCamRecorder::RecordedData dataType);

 private:
  /// Persists the captured frames on the disk.
  ProCamRecorder recorder_;
};

}}