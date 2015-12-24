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
{
  positions_.resize(ids.size(), cv::Point2i(0, 0));
  tracked_.resize(ids.size(), false);
  lastUpdate_.resize(ids.size(), std::chrono::steady_clock::now());
}

LaserDrawer::~LaserDrawer() {
}

// (*)
// (x, y) points sent by the procams are centered in top right corner,
// i.e. x increases to the left and y increases down. Thus, in order
// to transfer it to the coordinate system centered in the top left corner
// we leave the y as it is and invert the x.
//event.point_.x = 512 - event.point_.x - 1;

// TODO: Process event using 1. our matrices 2. the whole 3D reconstruction.
// TODO: Dynamically create list of laser colors and use it for responses.
void LaserDrawer::run() {
  std::cout << "Running the drawer" << std::endl;

  while (true) {
    const auto eventPair = stream_->poll();
    if (!eventPair.first) {
      break;
    }
    const auto event = eventPair.second;

    const auto position = event.getPosition();
    const auto proCamId = event.getProCamID();
    const auto kinect = system_->getProCam(proCamId);

    std::cout << "Received [x, y, depth]  " << position
              << " from procam# "           << proCamId << std::endl;

    // Project the point to the 3D space of the proCamId's kinect.
    auto position3D = projection::map3D(
        kinect->irCam_.proj,
        position.z / 1000.0f,
        position.x,
        position.y);

    // Consider the point from each projector's view.
    for (const auto projectorId : ids_) {
      const auto projector = system_->getProCam(projectorId);
      const auto pose = projector->poses.find(proCamId);
      const auto res = projector->effectiveProjRes_;

      // Check if the kinect is in the projector's group.
      if ( pose == projector->poses.end()) {
        continue;
      }

      std::vector<cv::Point3f> objectPoints { position3D };
      std::vector<cv::Point2f> imagePoints;

      // Project the point to the projector's screen space.
      cv::projectPoints(
          objectPoints,
          pose->second.rvec,
          pose->second.tvec,
          projector->projMat_,
          projector->projDist_,
          imagePoints);

      // Adjust the point for the projector view.
      cv::Point2i pointProj(
          projector->effectiveProjRes_.width - imagePoints[0].x - 1,
          imagePoints[0].y);

      // Discard out of view points.
      if (pointProj.x < 0 || pointProj.x >= res.width ||
          pointProj.y < 0 || pointProj.y >= res.height)
      {
        continue;
      }

      // If the time delay was too large, do not connect the lines.
      auto timeDiff = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - lastUpdate_[projectorId]);

      // Draw a segment if the laser is being tracked.
      if (tracked_[projectorId] && timeDiff < 750ms) {
        std::vector<std::pair<cv::Point2i, cv::Point2i>> path;
        path.emplace_back(positions_[projectorId], pointProj);
        connectionHandler_->updateLaser(projectorId, path, event.getColor());
      }

      // Remember the point.
      positions_[projectorId] = pointProj;
      tracked_[projectorId] = true;
      lastUpdate_[projectorId] = std::chrono::steady_clock::now();
    }
  }
}

