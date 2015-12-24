// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include "core/Projection.h"

#include "master/LaserDrawer.h"

using namespace dv::master;

using namespace std::literals;


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
  , lastUpdate_(std::chrono::steady_clock::now())
{
}

LaserDrawer::~LaserDrawer() {
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

    // (x, y) points sent by the procams are centered in top right corner,
    // i.e. x increases to the left and y increases down. Thus, in order
    // to transfer it to the coordinate system centered in the top left corner
    // we leave the y as it is and invert the x.
    // event.point_.x = 512 - event.point_.x - 1;

    auto newPosition = event.getPosition();

    // TODO: Process event using 1. our matrices 2. the whole 3D reconstruction.
    // TODO: Dynamically create list of laser colors and use it for responses.

    // DEMO
    // For demo - reject the corner with noise (we could in future probably
    // reject all extreme coordinates).
    if (newPosition.x != 0 && newPosition.y != 0) {
      if (tracked_) {
        handleEvent(event);
      }

      std::cout << "Received [x, y, depth]: " << newPosition << std::endl;

      tracked_ = true;
      position_ = newPosition;
      lastUpdate_ = std::chrono::steady_clock::now();
    }
  }
}

void LaserDrawer::handleEvent(const Event &e) {
  const auto p = e.getPosition();
  const auto kinectId = e.getProCamID();

  // If the time delay was too large, do not connect the lines.
  auto timeDiff = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now() - lastUpdate_
  );
  auto spaceDiff = std::sqrt(
      (position_.x - p.x) * (position_.x - p.x) +
      (position_.y - p.y) * (position_.y - p.y)
  );
  if (timeDiff > 750ms || spaceDiff > 200) {
    return;
  }

  // Find the projector that can see the detected point.
  for (const auto projectorId : ids_) {
    const auto projector = system_->getProCam(projectorId);
    const auto pose = projector->poses.find(kinectId);

    // Check if the projector can see the point from the event.
    if (pose == projector->poses.end()) {
      continue;
    }

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

    std::vector<cv::Point2f> imagePoints;

    // Project the points to projector space.
    cv::projectPoints(
        objectPoints,
        pose->second.rvec,
        pose->second.tvec,
        projector->projMat_,
        projector->projDist_,
        imagePoints);

    cv::Point2i p1(
        projector->effectiveProjRes_.width - imagePoints[0].x - 1,
        imagePoints[0].y);

    cv::Point2i p2(
        projector->effectiveProjRes_.width - imagePoints[1].x - 1,
        imagePoints[1].y);

    // Send a message to display the laser segment.
    std::vector<std::pair<cv::Point2i, cv::Point2i>> path;
    path.emplace_back(p1, p2);
    connectionHandler_->updateLaser(projectorId, path, e.getColor());
  }
}

