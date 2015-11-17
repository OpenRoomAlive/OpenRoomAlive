// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <iostream>

#include <boost/filesystem.hpp>

#include "master/MockConnectionHandler.h"

#include "core/Exception.h"

using namespace dv::master;
using namespace dv;

constexpr size_t kFrameTypes = 5;

enum FrameTypes
{
  COLOR          = 0,
  DEPTH          = 1,
  UNDIST         = 2,
  COLOR_BASLINE  = 3,
  DEPTH_BASELINE = 4
};

MockConnectionHandler::MockConnectionHandler(const std::string &path)
  : path_(path)
{
  namespace fs = boost::filesystem;

  // Find the number and generate IDs of procams for which images were recorded.
  ConnectionID currId = 0;

  for (fs::directory_iterator it(path); it != fs::directory_iterator(); ++it) {
    ids_.push_back(currId);
    // For every procam and every type of frame set the ID of the frame to be
    // returned next to 0.
    nextFrame_[currId++] = std::vector<size_t>(kFrameTypes, 0);
  }
}

std::vector<ConnectionID> MockConnectionHandler::waitForConnections(
  size_t count)
{
  if (count > ids_.size()) {
    throw EXCEPTION() << "Data was not recorded for the sufficient number of "
                         "procams.";
  }

  // Allow the recorded test data to be used with a smallr number of procams
  // than the recorded one */
  while (ids_.size() > count) {
    ids_.pop_back();
  }
  return ids_;
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

void MockConnectionHandler::stop() {}

void MockConnectionHandler::clearDisplays() {}

std::unordered_map<ConnectionID, CameraParams>
    MockConnectionHandler::getCamerasParams()
{
  // TODO: record this information in a file.
  throw EXCEPTION() << "Unsupported function.";
}

std::unordered_map<ConnectionID, DisplayParams>
    MockConnectionHandler::getDisplaysParams()
{
  // TODO: record this information in a file.
  throw EXCEPTION() << "Unsupported function.";
}

ConnectionHandler::FrameMap MockConnectionHandler::getColorImages() {
  throw EXCEPTION() << "Unsupported function.";
}

cv::Mat MockConnectionHandler::getDepthImage(ConnectionID id) {
  const auto frameId = nextFrame_[id][FrameTypes::DEPTH]++;

  boost::filesystem::path frameDest(path_);
  frameDest /= ("procam" + std::to_string(id));
  frameDest /= "depth_frames";
  frameDest /= ("frame" + std::to_string(frameId) + ".xml");

  std::cout << frameDest.string() << std::endl;
  cv::FileStorage fs(frameDest.string(), cv::FileStorage::READ);
  cv::Mat depthImage;
  fs["DepthImage"] >> depthImage;
  return (depthImage);
}

ConnectionHandler::FrameMap MockConnectionHandler::getDepthImages() {
  throw EXCEPTION() << "Unsupported function.";
}

ConnectionHandler::FrameMap MockConnectionHandler::getUndistortedColorImages() {
  throw EXCEPTION() << "Unsupported function.";
}

ConnectionHandler::FrameMap MockConnectionHandler::getColorBaselines() {
  throw EXCEPTION() << "Unsupported function.";
}

ConnectionHandler::FrameMap MockConnectionHandler::getDepthBaselines() {
  throw EXCEPTION() << "Unsupported function.";
}
