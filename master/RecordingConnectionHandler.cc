// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include "master/RecordingConnectionHandler.h"

RecordingConnectionHandler::RecordingConnectionHandler(
      uint16_t proCamPort,
      const std::shared_ptr<EventStream>& streamconst,
      const std::string &recordDirectory)
  : MasterConnectionHandler(proCamPort, streamconst)
  , recorder_(recordDirectory)
{
}

std::vector<ConnectionID> RecordingConnectionHandler::waitForConnections(
    size_t count)
{
  recorder_.createRecordDirectories(count);
  return MasterConnectionHandler::waitForConnections(count);
}

FrameMap RecordingConnectionHandler::getColorImages() {
  FrameMap colorImages = MasterConnectionHandler::getColorImages();
  // TODO: save the color images
  return colorImages;
}

cv::Mat RecordingConnectionHandler::getDepthImage(ConnectionID id) {
  cv::Mat depthImage = MasterConnectionHandler::getDepthImage(id);
  // TODO: save the depth image
  return depthImage;
}

FrameMap RecordingConnectionHandler::getDepthImages() {
  FrameMap depthImages = MasterConnectionHandler::getDepthImages();
  // TODO: save the depth images
  return depthImages;
}

FrameMap RecordingConnectionHandler::getUndistortedColorImages()
{
  FrameMap undistortedImages =
      MasterConnectionHandler::getUndistortedColorImages();
  // TODO: save the undistorted images
  return undistortedImages;
}

FrameMap RecordingConnectionHandler::getColorBaselines() {
  FrameMap colorBaselines = MasterConnectionHandler::getColorBaselines();
  // TODO: save the color baselines.
  return colorBaselines;
}

FrameMap RecordingConnectionHandler::getDepthBaselines() {
  FrameMap depthBaselines = MasterConnectionHandler::getDepthBaselines();
  // TODO: save the depth baselines.
  return depthBaselines;
}

FrameMap RecordingConnectionHandler::getDepthVariances() {
  FrameMap depthVariances = MasterConnectionHandler::getDepthVariances();
  // TODO: save the depth variances
  return depthVariances;
}

cv::Mat RecordingConnectionHandler::undistort(
    ConnectionID id, const cv::Mat &imageHD)
{
  cv::Mat undistortedHD = MasterConnectionHandler::undistort(id, imageHD);
  // TODO: save the undistorted HD image
  return undistortedHD;
}