// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include "core/Geometry.h"
#include "master/ConnectionHandler.h"
#include "master/PointCloud.h"
#include "master/ProCam.h"
#include "master/ProCamSystem.h"

#include "master/EnviromentDrawer.h"


using namespace dv::master;

EnviromentDrawer::EnviromentDrawer(
    const std::vector<ConnectionID>& ids,
    const std::shared_ptr<ProCamSystem> system,
    const boost::shared_ptr<ConnectionHandler> connectionHandler,
    const std::shared_ptr<PointCloud> pointCloud)
  : ids_(ids)
  , system_(system)
  , connectionHandler_(connectionHandler)
  , pointCloud_(pointCloud)
{
}

EnviromentDrawer::~EnviromentDrawer() {
}

void EnviromentDrawer::findFloor() {
  std::vector<cv::Point3f> points;
  for (const auto v : pointCloud_->getPoints()) {
    points.emplace_back(v.position);
  }

  // Find planes in the point cloud.
  const auto planes = fitPlanes(points);

  // Since y in the coordinate system of the point cloud points downwards,
  // the highest horizontal plane corresponds to the floor.
  floor_.d = std::numeric_limits<float>::min();

  for (const auto &plane : planes) {
    if (plane.d > floor_.d) {
      floor_ = plane;
    }
  }
}

void EnviromentDrawer::drawCarpet() {
  // Extract the relative poses.
  auto relativePoses = pointCloud_->getPoses();

  // Find the points that belong to the floor.
  const auto floorPoints = findPointsInPlane(pointCloud_->getPoints(), floor_);

  // Group the floor points by the views used to acquire them.
  std::unordered_map<ConnectionID, std::vector<cv::Point3f>> viewsPoints;
  for (const auto &vertex : floorPoints) {
    viewsPoints[vertex.viewId].push_back(vertex.position);
  }

  // Transform the points to the coordinate systems of their views.
  std::unordered_map<ConnectionID, std::vector<cv::Point3f>> transformedPoints;
  for (const auto vp : viewsPoints) {
    const auto viewId = vp.first;
    const auto points = vp.second;
    const auto relativePose = relativePoses[viewId];

    // Compute the inverse rotation vector.
    cv::Mat rmat;
    cv::Rodrigues(relativePose.rvec, rmat);
    cv::Mat invRmat = rmat.inv();

    // Compute the inverse translation vector.
    cv::Mat invTvec = -relativePose.tvec;

    // Create temporaries used for performing transformations.
    cv::Point3f tp;
    cv::Mat tm(invTvec.rows, invTvec.cols, invTvec.type());

    // Transform the points.
    for (const auto p : points) {
      tm.at<double>(0, 0) = p.x;
      tm.at<double>(1, 0) = p.y;
      tm.at<double>(2, 0) = p.z;

      // Transform the point to the view.
      tm = invRmat * tm + invTvec;

      tp.x = tm.at<double>(0, 0);
      tp.y = tm.at<double>(1, 0);
      tp.z = tm.at<double>(2, 0);

      transformedPoints[viewId].push_back(tp);
    }
  }

  // For each view project its floor points to the image space of its projector.
  std::unordered_map<ConnectionID, std::vector<cv::Point2f>> projectorPoints;
  for (const auto vp : transformedPoints) {
    const auto viewId = vp.first;
    const auto points = vp.second;
    const auto projector = system_->getProCam(viewId);
    const auto pose = projector->poses_[viewId];

    // Project the points.
    cv::projectPoints(
        points,
        pose.rvec,
        pose.tvec,
        projector->projMat_,
        projector->projDist_,
        projectorPoints[viewId]);
  }

  // Send messages to the projectors to draw the carpet.
  for (const auto vp : projectorPoints) {
    // TODO
  }
}

