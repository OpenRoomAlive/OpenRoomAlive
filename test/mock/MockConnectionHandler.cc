// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <iostream>
#include <fstream>

#include <boost/make_shared.hpp>
#include <boost/filesystem.hpp>

#include <thrift/protocol/TJSONProtocol.h>
#include <thrift/transport/TFileTransport.h>

#include "core/Conv.h"
#include "core/Exception.h"
#include "test/mock/MockConnectionHandler.h"

using namespace dv::master;
using namespace dv;


MockConnectionHandler::MockConnectionHandler(
    const boost::filesystem::path &path)
  : path_(path)
  , count_(countDirs(path))
{
 for (size_t id = 0; id < count_; id++) {
    nextFrame_.emplace_back();
  }
}

std::vector<ConnectionID> MockConnectionHandler::waitForConnections(
    size_t count)
{
  if (count > count_) {
    // Data was captured for fewer proCams than the user expects to connect.
    throw EXCEPTION() << "Not enough captured data.";
  }

  count_ = count;
  std::vector<ConnectionID> ids;

  for (size_t id = 0; id < count_; ++id) {
    ids.push_back(id);
  }
  return ids;
}

void MockConnectionHandler::displayGrayCode(
    ConnectionID id,
    Orientation::type orientation,
    int16_t level,
    bool invertedGrayCode)
{
  (void) id;
  (void) orientation;
  (void) level;
  (void) invertedGrayCode;
}

void MockConnectionHandler::displayWhite(ConnectionID id) {
  (void) id;
}

void MockConnectionHandler::clearDisplay(ConnectionID id) {
  (void) id;
}

void MockConnectionHandler::stop() {
}

void MockConnectionHandler::clearDisplays() {
}

ConnectionHandler::FrameMap MockConnectionHandler::getGrayscaleImages() {
  return loadFrames(ProCamRecorder::RecordedData::GRAYSCALE);
}

cv::Mat MockConnectionHandler::getDepthImage(ConnectionID id) {
  return loadFrame(id, ProCamRecorder::RecordedData::DEPTH);
}

ConnectionHandler::FrameMap MockConnectionHandler::getDepthImages() {
  return loadFrames(ProCamRecorder::RecordedData::DEPTH);
}

ConnectionHandler::FrameMap MockConnectionHandler::getUndistortedColorImages() {
  return loadFrames(ProCamRecorder::RecordedData::UNDISTORTED);
}

ConnectionHandler::FrameMap MockConnectionHandler::getColorBaselines() {
  return loadFrames(ProCamRecorder::RecordedData::COLOR_BASELINE);
}

ConnectionHandler::FrameMap MockConnectionHandler::getDepthBaselines() {
  return loadFrames(ProCamRecorder::RecordedData::DEPTH_BASELINE);
}

ConnectionHandler::FrameMap MockConnectionHandler::getDepthVariances() {
  return loadFrames(ProCamRecorder::RecordedData::DEPTH_VARIANCE);
}

cv::Mat MockConnectionHandler::undistort(
    ConnectionID id,
    const cv::Mat &imageHD)
{
  (void) imageHD;

  return loadFrame(id, ProCamRecorder::RecordedData::UNDISTORTED_HD);
}

ConnectionHandler::ParamMap MockConnectionHandler::getParams() {
  ParamMap params;
  for (ConnectionID id = 0; id < count_; id++) {
    params[id] = loadParam(id);
  }
  return params;
}

size_t MockConnectionHandler::countDirs(const boost::filesystem::path path) {
  namespace fs = boost::filesystem;

  // Number of subdirectories of the directory path directory.
  size_t count = 0;

  fs::directory_iterator end = fs::directory_iterator();
  for (fs::directory_iterator it(path); it != end; ++it) {
    count++;
  }

  return count;
}

void MockConnectionHandler::startLaserDetection() {
}

void MockConnectionHandler::updateLaser(
      ConnectionID id,
      const std::vector<std::pair<cv::Point2i, cv::Point2i>> &path,
      const cv::Scalar &color)
{
  (void) id;
  (void) path;
  (void) color;
}

cv::Mat MockConnectionHandler::loadFrame(
    ConnectionID id,
    ProCamRecorder::RecordedData dataType)
{
  auto frameDir = ProCamRecorder::kDataDirNames.find(dataType)->second;

  boost::filesystem::path framePath(path_);
  framePath /= (ProCamRecorder::kProCamDir + std::to_string(id));
  framePath /= frameDir;
  framePath /= frameName(id, dataType);

  return(cv::imread(framePath.string(), CV_LOAD_IMAGE_UNCHANGED));
}

ConnectionHandler::FrameMap MockConnectionHandler::loadFrames(
    ProCamRecorder::RecordedData dataType)
{
  FrameMap loadedFrames;
  for (ConnectionID id = 0; id < count_; id++) {
    loadedFrames[id] = loadFrame(id, dataType);
  }
  return loadedFrames;
}

ProCamParam MockConnectionHandler::loadParam(ConnectionID id) {
  namespace tt = apache::thrift::transport;
  namespace tp = apache::thrift::protocol;

  // Build a path to the directory in which camera paramters are saved.
  boost::filesystem::path source(path_);
  source /= (ProCamRecorder::kProCamDir + std::to_string(id));
  source /= "param.json";

  auto transport = boost::make_shared<tt::TFileTransport>(source.string());
  auto protocol = boost::make_shared<tp::TJSONProtocol>(
      boost::static_pointer_cast<tp::TTransport>(transport));

  ProCamParam param;
  param.read(protocol.get());
  return param;
}

std::string MockConnectionHandler::frameName(
    ConnectionID id,
    ProCamRecorder::RecordedData dataType)
{
  std::string ext;
  switch (dataType) {
    case ProCamRecorder::RecordedData::DEPTH:
    case ProCamRecorder::RecordedData::DEPTH_BASELINE:
    case ProCamRecorder::RecordedData::DEPTH_VARIANCE:
      ext = "exr";
      break;
    default :
      ext = "png";
      break;
  }
  return "frame" + std::to_string(nextFrame_[id][dataType]++) + "." + ext;
}
