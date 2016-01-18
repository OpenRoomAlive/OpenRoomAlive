// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <memory>
#include <unordered_map>

#include <opencv2/opencv.hpp>

#include "core/Types.h"


namespace dv { namespace master {

class ProCamSystem;

/**
 * Point cloud.
 */
class PointCloud {
 public:
  PointCloud(
      const std::vector<ConnectionID> &ids,
      const std::shared_ptr<ProCamSystem> system);
  ~PointCloud();

  /**
   * Constructs the point cloud for the procam system.
   */
  void construct();

  /**
   * Returns the points in the point cloud.
   */
  const std::vector<Vertex> getPoints() const { return points_; }

  /**
   * Returns the centroid of the points in the point cloud.
   */
  const cv::Point3f getCentroid() const { return centroid_; }

  /**
   * Returns the poses.
   */
  const std::unordered_map<ConnectionID, CameraPose> getPoses() {
    return poses_;
  }

  /**
   * Adds points from a view to the point cloud.
   *
   * @param viewId   Id of the ProCam used to capture the view.
   * @param depthMap Depth map recorder by the camera.
   * @param colorMap Undistorted color image.
   * @param depthCamera Calibration matrix and distortion coefficients.
   * @param relativePose Relative pose from the view to the origin.
   */
  void addView(
      ConnectionID viewId,
      const cv::Mat &depthMap,
      const cv::Mat &colorMap,
      const CameraModel &depthCamera,
      const CameraPose &relativePose);

 private:
  /// ProCam ids.
  const std::vector<ConnectionID> ids_;
  /// ProCam system.
  const std::shared_ptr<ProCamSystem> system_;
  /// Point data.
  std::vector<Vertex> points_;
  /// Centroid of the points.
  cv::Point3f centroid_;
  /// Relative poses between the center of the point cloud and the views.
  std::unordered_map<ConnectionID, CameraPose> poses_;
};

}}
