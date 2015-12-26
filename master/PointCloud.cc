// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include "core/Geometry.h"
#include "core/Projection.h"
#include "master/PointCloud.h"

using namespace dv::master;
using namespace dv::projection;

PointCloud::PointCloud(
    const std::vector<ConnectionID> &ids,
    const std::shared_ptr<ProCamSystem> system)
  : ids_(ids)
  , system_(system)
{
}

PointCloud::~PointCloud() {
}

void PointCloud::construct() {
  // Pick a procam and take its projector's world coordinate space as the
  // coordinate space of the point cloud.
  auto procam = system_->getProCam(ids_[0]);

  // Add procam's projector group views.
  for (const auto &idPose : procam->poses_) {
    const auto kinect = system_->getProCam(idPose.first);
    const auto relativePose = idPose.second;

    // Add the points captured from the kinect's view.
    addView(
        kinect->depthBaseline_,
        kinect->colorBaseline_,
        kinect->irCam_,
        relativePose);
  }
}

void PointCloud::addView(
    const cv::Mat &depthMap,
    const cv::Mat &colorMap,
    const CameraModel &depthCamera,
    const CameraPose &relativePose)
{
  cv::Mat tvec = relativePose.tvec;
  // Construct the rotation matrix.
  cv::Mat rotation;
  cv::Rodrigues(relativePose.rvec, rotation);

  // Construct a vector used for performing transformations.
  cv::Mat p(tvec.rows, tvec.cols, tvec.type());

  // Construct the 3D points from the depth map.
  for (auto y = 0; y < depthMap.rows; ++y) {
    for (auto x = 0; x < depthMap.cols; ++x) {
      // Convert the depth to meters.
      auto depth = depthMap.at<float>(y, x) / kMilimetersToMeters;
      auto colorByte = colorMap.at<cv::Vec4b>(y, x);
      cv::Point3f color(
          colorByte[2] / 255.0f, 
          colorByte[1] / 255.0f,
          colorByte[0] / 255.0f
      );

      // TODO: consider filtering on variance as well
      // Filter out the noisy points.
      if (equals(depth, 0.0f)) {
        continue;
      }

      // Compute the 3D point in the view.
      auto point3D = map3D(depthCamera.calib, depth, x, y);

      // Convert cv::Point3f to cv::Mat
      p.at<double>(0, 0) = point3D.x;
      p.at<double>(1, 0) = point3D.y;
      p.at<double>(2, 0) = point3D.z;

      // Transform the point to the point cloud origin.
      p = rotation * p + relativePose.tvec;

      // Convert back to cv::Point3f.
      point3D.x = p.at<double>(0, 0);
      // Since +y in our coordinate system is down, invert y.
      point3D.y = -p.at<double>(1, 0);
      point3D.z = p.at<double>(2, 0);

      // Save the point.
      points_.emplace_back(point3D, color);
    }
  }

  // Compute the centroid.
  centroid_ = {0.0f};
  for (const auto &point : points_) {
    centroid_ += point.first;
  }
  centroid_ *= 1.0f / points_.size();

  // Save the camera params.
  intrinsics_.emplace_back(depthCamera);
  // TODO: compute the absolute pose and save it.
  poses_.emplace_back(relativePose);
}

