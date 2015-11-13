// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include "core/Projection.h"

namespace dv { namespace projection {


cv::Point2f project(
    const cv::Mat& camera,
    float k1,
    float k2,
    const cv::Point3f& point)
{
  const float xp = point.x / point.z;
  const float yp = point.y / point.z;

  const float fx = camera.at<float>(0, 0);
  const float fy = camera.at<float>(1, 1);
  const float cx = camera.at<float>(0, 2);
  const float cy = camera.at<float>(1, 2);

  const float r2 = xp * xp + yp * yp;
  const float d = (1 + k1 * r2) / (1 + k2 * r2);

  const float xpp = xp * d;
  const float ypp = yp * d;

  const float u = fx * xpp + cx;
  const float v = fy * ypp + cy;

  return cv::Point2f(u, v);
}

cv::Point3f map3D(const cv::Mat& camera, float depth, size_t r, size_t c) {
  // Principal points of the camera.
  const double cx = camera.at<double>(0, 2);
  const double cy = camera.at<double>(1, 2);

  // Focal points.
  const double fx = camera.at<double>(0, 0);
  const double fy = camera.at<double>(1, 1);

  const double x =
      ((static_cast<double>(c) - cx) * static_cast<double>(depth)) / fx;
  // TODO(ilijar) : T51
  const double y =
      ((static_cast<double>(424 - r - 1) - cy) * static_cast<double>(depth)) / fy;
  const double z = static_cast<double>(depth);

  return cv::Point3f(x, y, z);
}

}}

