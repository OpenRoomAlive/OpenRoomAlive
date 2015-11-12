// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <limits>
#include <random>

#include <boost/math/constants/constants.hpp>

#include "core/Geometry.h"

namespace dv {


constexpr size_t SPLIT_PHI   = 10;
constexpr size_t SPLIT_THETA = 20;
constexpr size_t SPLIT_R     = 30;


Plane planeFit(
    const std::vector<cv::Point3f> &points,
    unsigned maxIter,
    unsigned minPoints,
    float eps)
{
  const float PI = boost::math::constants::pi<float>();

  // Compute the center point of the dataset.
  cv::Point3f center(0.0f);
  for (const auto &point : points) {
    center += point;
  }
  center = center * (1.0f /points.size());

  // Compute the range for r.
  auto maxR = std::numeric_limits<float>::min();
  for (const auto &point : points) {
    auto diff = point - center;
    maxR = std::max(maxR, std::sqrt(diff.dot(diff)));
  }

  // 3D hough transform.
  // R \in [-maxR to maxR], p \in [0 to PI], theta \in [0 to 2PI]
  unsigned hough[SPLIT_PHI + 1][SPLIT_THETA + 1][SPLIT_R + 1];
  for (size_t p = 0; p <= SPLIT_PHI; ++p) {
    for (size_t t = 0; t <= SPLIT_THETA; ++t) {
      for (size_t r = 0; r <= SPLIT_R; ++r) {
        hough[p][t][r] = 0;
      }
    }
  }

  // The plane's equation is expressed using polar coordinates:
  //
  // nx = cos(phi) * cos(theta)
  // ny = cos(phi) * sin(theta)
  // nz = sin(phi)
  //
  // A point is on the plane iff:
  //
  // nx * x + ny * y + nz * z - d = 0
  //
  // So our task is to fix all possible phi and theta and find the corresponding
  // d for all points and vote for it in the 3D matrix. Polar coordinates are
  // used because sin and cos are periodic, so phi and theta can be sensibly
  // subdivided into a finite number of intervals.
  for (const auto &point : points) {
    auto diff = point - center;
    for (size_t p = 0; p < SPLIT_PHI; ++p) {
      const float phi = static_cast<float>(p) / SPLIT_PHI * PI;
      for (size_t t = 0; t < SPLIT_THETA; ++t) {
        const float theta = static_cast<float>(t) / SPLIT_THETA * (2.0f * PI);
        const float r =
            std::cos(phi) * std::cos(theta) * diff.x +
            std::cos(phi) * std::sin(theta) * diff.y +
            std::sin(phi) * diff.z;
        const size_t index = (r / (2 * maxR) + 0.5f) * SPLIT_R;
        hough[p][t][index]++;
      }
    }
  }

  // Find the bucket with the most votes.
  size_t idxP = 0, idxT = 0, idxR = 0;
  for (size_t p = 0; p < SPLIT_PHI; ++p) {
    for (size_t t = 0; t < SPLIT_THETA; ++t) {
      for (size_t r = 0; r < SPLIT_R; ++r) {
        if (hough[p][t][r] > hough[idxP][idxT][idxR]) {
          idxP = p;
          idxT = t;
          idxR = r;
        }
      }
    }
  }

  // Find the parameters of that plane.
  const float phi = static_cast<float>(idxP) / SPLIT_PHI * PI;
  const float theta = static_cast<float>(idxT) / SPLIT_THETA * (2.0f * PI);
  const float r = (static_cast<float>(idxR) / SPLIT_R - 0.5f) * 2.0f * maxR;

  // TODO(nand): Refine using RANSAC
  (void) maxIter;
  (void) minPoints;
  (void) eps;

  const float nx = std::cos(phi) * std::cos(theta);
  const float ny = std::cos(phi) * std::sin(theta);
  const float nz = std::sin(phi);
  const float l = std::sqrt(nx * nx + ny * ny + nz * nz + r * r);

  return { nx / l, ny / l, nz / l, r / l};
}

}
