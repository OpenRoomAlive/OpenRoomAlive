// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include "master/RecordingConnectionHandler.h"

using namespace dv;
using namespace dv::master;

RecordingConnectionHandler::RecordingConnectionHandler(
      uint16_t proCamPort,
      const std::shared_ptr<EventStream>& streamconst,
      const std::string &recordDirectory)
  : MasterConnectionHandler(proCamPort, streamconst)
  , recorder_(recordDirectory)
{
}

RecordingConnectionHandler::~RecordingConnectionHandler() {
}

std::vector<ConnectionID> RecordingConnectionHandler::waitForConnections(
    size_t count)
{
  recorder_.createRecordDirectories(count);
  return MasterConnectionHandler::waitForConnections(count);
}

ConnectionHandler::FrameMap RecordingConnectionHandler::getGrayscaleImages() {
  FrameMap colorImages = MasterConnectionHandler::getGrayscaleImages();
  recordAll(colorImages, ProCamRecorder::RecordedData::GRAYSCALE);
  return colorImages;
}

cv::Mat RecordingConnectionHandler::getDepthImage(ConnectionID id) {
  cv::Mat depthImage = MasterConnectionHandler::getDepthImage(id);
  recorder_.saveFrame(depthImage, id, ProCamRecorder::RecordedData::DEPTH);
  return depthImage;
}

ConnectionHandler::FrameMap RecordingConnectionHandler::getDepthImages() {
  FrameMap depthImages = MasterConnectionHandler::getDepthImages();
  recordAll(depthImages, ProCamRecorder::RecordedData::DEPTH);
  return depthImages;
}

ConnectionHandler::FrameMap
    RecordingConnectionHandler::getUndistortedColorImages()
{
  FrameMap undistortedImages =
      MasterConnectionHandler::getUndistortedColorImages();
  recordAll(undistortedImages, ProCamRecorder::RecordedData::UNDISTORTED);
  return undistortedImages;
}

ConnectionHandler::FrameMap RecordingConnectionHandler::getColorBaselines() {
  FrameMap colorBaselines = MasterConnectionHandler::getColorBaselines();
  recordAll(colorBaselines, ProCamRecorder::RecordedData::COLOR_BASELINE);
  return colorBaselines;
}

ConnectionHandler::FrameMap RecordingConnectionHandler::getDepthBaselines() {
  FrameMap depthBaselines = MasterConnectionHandler::getDepthBaselines();
  recordAll(depthBaselines, ProCamRecorder::RecordedData::DEPTH_BASELINE);
  return depthBaselines;
}

ConnectionHandler::FrameMap RecordingConnectionHandler::getDepthVariances() {
  FrameMap depthVariances = MasterConnectionHandler::getDepthVariances();
  recordAll(depthVariances, ProCamRecorder::RecordedData::DEPTH_VARIANCE);
  return depthVariances;
}

ConnectionHandler::ParamMap RecordingConnectionHandler::getParams()
{
  auto cameraParams = MasterConnectionHandler::getParams();

  for (auto idParams : cameraParams) {
    recorder_.saveParam(idParams.second, idParams.first);
  }
  return cameraParams;
}

void RecordingConnectionHandler::recordAll(
    const FrameMap &capturedFrames,
    ProCamRecorder::RecordedData dataType)
{
  for (auto idFrame : capturedFrames) {
    recorder_.saveFrame(idFrame.second, idFrame.first, dataType);
  }
}
