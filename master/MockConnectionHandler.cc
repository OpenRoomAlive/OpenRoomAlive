// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <iostream>
#include <fstream>

#include <boost/filesystem.hpp>

#include "core/Conv.h"
#include "core/Exception.h"
#include "master/MockConnectionHandler.h"

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

ConnectionHandler::FrameMap MockConnectionHandler::getColorImages() {
  return loadFrames(ProCamRecorder::RecordedData::COLOR);
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

std::unordered_map<ConnectionID, CameraParams>
    MockConnectionHandler::getCamerasParams()
{
  std::unordered_map<ConnectionID, CameraParams> params;

  for (ConnectionID id = 0; id < count_; id++) {
    params[id] = loadCameraParams(id);
  }
  return params;
}

std::unordered_map<ConnectionID, DisplayParams>
    MockConnectionHandler::getDisplaysParams()
{
  std::unordered_map<ConnectionID, DisplayParams> params;

  for (ConnectionID id = 0; id < count_; id++) {
    params[id] = loadDisplayParams(id);
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

CameraParams MockConnectionHandler::loadCameraParams(ConnectionID id) {
  // Build a path to the directory in which camera paramters are saved.
  boost::filesystem::path paramsPath(path_);
  paramsPath /= (ProCamRecorder::kProCamDir + std::to_string(id));
  paramsPath /= ProCamRecorder::kDataDirNames.find(
      ProCamRecorder::RecordedData::CAMERA_PARAMS)->second;
  paramsPath /= "cameraParams.xml";

  std::cout << paramsPath.string() << std::endl;

  CameraParams params;
  cv::Mat colCamParams, depthCamParams, distCoefs;

  // Load the camera paramters from a XML file.
  cv::FileStorage fs(paramsPath.string(), cv::FileStorage::READ);

  fs[ProCamRecorder::kColorCamParams] >> colCamParams;
  fs[ProCamRecorder::kDepthCamParams] >> depthCamParams;
  fs[ProCamRecorder::kDisplayParams]  >> distCoefs;
  fs.release();

  // Convert the paramters to thrift format.
  params.colorCamMat = dv::conv::cvMatToThriftCamMat(colCamParams);
  params.irCamMat    = dv::conv::cvMatToThriftCamMat(depthCamParams);
  params.irDist      = dv::conv::cvMatToThriftDistCoef(distCoefs);

  return params;
}

DisplayParams MockConnectionHandler::loadDisplayParams(ConnectionID id) {
  // Build a path to the directory in which display paramters are saved.
  boost::filesystem::path paramsPath(path_);
  paramsPath /= (ProCamRecorder::kProCamDir + std::to_string(id));
  paramsPath /= ProCamRecorder::kDataDirNames.find(
      ProCamRecorder::RecordedData::DISPLAY_PARAMS)->second;
  paramsPath /= "displayParams.txt";

  DisplayParams params;
  // Retrieve the paramters from a file.
  std::ifstream paramsFile(paramsPath.string());
  paramsFile >> params.effectiveRes.width;
  paramsFile >> params.effectiveRes.height;
  paramsFile.close();

  return params;
}

std::string MockConnectionHandler::frameName(
    ConnectionID id,
    ProCamRecorder::RecordedData dataType)
{
  std::string ext;
  switch (dataType) {
    case ProCamRecorder::RecordedData::DEPTH          :
    case ProCamRecorder::RecordedData::DEPTH_BASELINE :
    case ProCamRecorder::RecordedData::DEPTH_VARIANCE :
      ext = "exr";
      break;
    default :
      ext = "png";
  }
  return "frame" + std::to_string(nextFrame_[id][dataType]++) + "." + ext;
}
