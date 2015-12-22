// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include "core/Projection.h"

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
  , tracked_(false)
{
}

LaserDrawer::~LaserDrawer(){
}

void LaserDrawer::run() {
  std::cout << "Running the drawer" << std::endl;

  // TODO: Keep position (and other data) per detected laser (i.e. detected
  // laser color)
  while (true) {
    auto eventPair = stream_->poll();
    if (!eventPair.first) {
      break;
    }
    auto event = eventPair.second;
    auto newPosition = event.getPosition();

    // x = column, y = row
    auto column = newPosition.x;
    auto row = newPosition.y;
    // x = row, y = column
    newPosition.x = row;
    newPosition.y = column;

    // TODO: Process event using 1. our matrices 2. the whole 3D reconstruction.
    // TODO: Dynamically create list of laser colors and use it for responses.

    // DEMO
    // For demo - reject the corner with noise (we could in future probably
    // reject all extreme coordinates).
    if (newPosition.x != 0 && newPosition.y != 0) {
      if (tracked_) {
        handleEvent(event);
      }

      /*
      std::cout << "Received: " << newPosition << std::endl;
      std::cout << "Inverted: " << "r: " << newPosition.y
                                << "c: " << newPosition.x
                                << std::endl;
      */

      tracked_ = true;
      position_ = newPosition;
    }
  }
}

void LaserDrawer::handleEvent(const Event &e) {
  auto p = e.getPosition();

  // x = column, y = row
  auto row = p.y;
  auto column = p.x;

  // x = row, y = column
  p.x = row;
  p.y = column;

  const auto kinectId = e.getProCamID();

  // Find the projector that can see the detected point.
  for (const auto projectorId : ids_) {
    const auto projector = system_->getProCam(projectorId);
    const auto pose = projector->poses.find(kinectId);

    // Check if the projector can see the point from the event.
    if (pose == projector->poses.end()) {
      continue;
    }

    std::cout << "Received:" << std::endl;
    std::cout << "Prev: " << position_ << " Curr: " << p << std::endl;

    std::vector<cv::Point3f> objectPoints { position_, p };

    // Consturct 3D points in kinect world space.
    auto kinect = system_->getProCam(kinectId);

    auto d1 = position_.z / 1000.0f;
    auto d2 = p.z / 1000.0f;

    objectPoints[0] = projection::map3D(
        kinect->irCam_.proj,
        d1,
        objectPoints[0].x,
        objectPoints[0].y);

    objectPoints[1] = projection::map3D(
        kinect->irCam_.proj,
        d2,
        objectPoints[1].x,
        objectPoints[1].y);

    std::cout << "3D:" << std::endl;
    std::cout << "Prev: "
              << objectPoints[0]
              << " Curr: "
              << objectPoints[1] << std::endl;

    std::vector<cv::Point2f> imagePoints;

    // Project the points to projector space.
    cv::projectPoints(
        objectPoints,
        pose->second.rvec,
        pose->second.tvec,
        projector->projMat_,
        projector->projDist_,
        imagePoints);

    /*
    cv::Point2i p1(imagePoints[0].y, imagePoints[0].x);
    cv::Point2i p2(imagePoints[1].y, imagePoints[1].x);
    */

    cv::Point2i p1(
        projector->effectiveProjRes_.width - imagePoints[0].x - 1,
        projector->effectiveProjRes_.height - imagePoints[0].y - 1);

    cv::Point2i p2(
        projector->effectiveProjRes_.width - imagePoints[1].x - 1,
        projector->effectiveProjRes_.height - imagePoints[1].y - 1);

    /*
    cv::Point2i p1(
        imagePoints[0].x,
        projector->effectiveProjRes_.height - imagePoints[0].y - 1);

    cv::Point2i p2(
        imagePoints[1].x,
        projector->effectiveProjRes_.height - imagePoints[1].y - 1);
    */

    std::cout << "Projected: " << std::endl;
    std::cout << "Prev: " << p1
              << " Curr: " << p2
              << std::endl;

    // Send a message to display the laser segment.
    std::vector<std::pair<cv::Point2i, cv::Point2i>> path;
    path.emplace_back(p1, p2);
    connectionHandler_->updateLaser(projectorId, path, e.getColor());
  }
}

