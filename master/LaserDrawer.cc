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
  // TODO: Keep position (and other data) per detected laser (i.e. detected
  // laser color)
  cv::Point3f position;
  bool tracked = false;
  while (true) {
    auto eventPair = stream_->poll();
    if (!eventPair.first) {
      break;
    }
    auto event = eventPair.second;
    auto newPosition = event.getPosition();
    // TODO: Process event using 1. our matrices 2. the whole 3D reconstruction.
    // TODO: Dynamically create list of laser colors and use it for responses.

    // DEMO
    // For demo - reject the corner with noise (we could in future probably
    // reject all extreme coordinates).
    if (newPosition.x != 0 && newPosition.y != 0) {
      if (tracked) {
        std::vector<std::pair<cv::Point2i, cv::Point2i>> path;
        path.emplace_back(
            cv::Point2i(position.x, position.y),
            cv::Point2i(newPosition.x, newPosition.y));
        connectionHandler_->updateLaser(
            event.getProCamID(),
            path,
            event.getColor());
      }

      tracked = true;
      position.x = newPosition.x;
      position.y = newPosition.y;
    }
  }
}

