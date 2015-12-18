// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include "master/LaserDrawer.h"

using namespace dv::master;


LaserDrawer::LaserDrawer(
    const std::vector<ConnectionID>& ids,
    const std::shared_ptr<ProCamSystem> system,
    const std::shared_ptr<EventStream> stream,
    const boost::shared_ptr<ConnectionHandler> connectionHandler)
  : ids_(ids)
  , system_(system)
  , stream_(stream)
  , connectionHandler_(connectionHandler)
{
}

LaserDrawer::~LaserDrawer(){
}

void LaserDrawer::run() {
  while (true) {
    auto eventPair = stream_->poll();
    if (!eventPair.first) {
      break;
    }
    auto event = eventPair.second;
    // TODO: Process event using 1. our matrices 2. the whole 3D reconstruction.
    // TODO: Dynamically create list of laser colors and use it for responses.

    // DEMO
    cv::Scalar color(0, 255, 0);
    std::vector<std::pair<cv::Point2i, cv::Point2i>> path;
    auto x1 = rand() % 2000;
    auto y1 = rand() % 1000;
    auto x2 = rand() % 2000;
    auto y2 = rand() % 1000;
    auto x3 = rand() % 2000;
    auto y3 = rand() % 1000;

    path.emplace_back(cv::Point2i(x1, y1), cv::Point2i(x2, y2));
    path.emplace_back(cv::Point2i(x2, y2), cv::Point2i(x3, y3));

    connectionHandler_->updateLaser(event.getProCamID(), path, color);
  }
}

