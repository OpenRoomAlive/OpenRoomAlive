// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <algorithm>
#include <limits>
#include <random>

#include <boost/math/constants/constants.hpp>

#include <glm/ext.hpp>

#include "core/Exception.h"
#include "core/Geometry.h"


constexpr size_t kSplitPhi = 10;
constexpr size_t kSplitTheta = 20;
constexpr size_t kSplitR = 30;
constexpr float kEps = 1e-5;


static float findMedian(std::vector<float> &numbers) {
  std::nth_element(
      numbers.begin(),
      numbers.begin() + numbers.size() / 2,
      numbers.end());
  return numbers[numbers.size() / 2];
}

namespace dv {

Plane planeFit(
    const std::vector<cv::Point3f> &points,
    unsigned maxIter,
    unsigned minPoints,
    float eps)
{
  constexpr auto PI = boost::math::constants::pi<float>();

  // Compute the center point. The hough transform is performed in this
  // system in order to make best use of the buckets on R.
  const auto center = findCenter(points);

  // Compute the range for r.
  auto maxR = std::numeric_limits<float>::min();
  for (const auto &point : points) {
    const auto diff = point - center;
    maxR = std::max(maxR, std::sqrt(diff.dot(diff)));
  }

  // 3D hough transform.
  // R \in [-maxR to maxR], p \in [0 to PI], theta \in [0 to 2PI]
  unsigned hough[kSplitPhi + 1][kSplitTheta + 1][kSplitR + 1];
  for (size_t p = 0; p <= kSplitPhi; ++p) {
    for (size_t t = 0; t <= kSplitTheta; ++t) {
      for (size_t r = 0; r <= kSplitR; ++r) {
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
    const auto diff = point - center;
    for (size_t p = 0; p <= kSplitPhi; ++p) {
      const float phi = static_cast<float>(p) / kSplitPhi * PI;
      for (size_t t = 0; t <= kSplitTheta; ++t) {
        const float theta = static_cast<float>(t) / kSplitTheta * (2.0f * PI);
        const float r =
            std::cos(phi) * std::cos(theta) * diff.x +
            std::cos(phi) * std::sin(theta) * diff.y +
            std::sin(phi) * diff.z;

        // TODO(nand): mirror normals so all R's are positive.
        const size_t index = (r / (2 * maxR) + 0.5f) * kSplitR;
        if (index <= kSplitR) {
          hough[p][t][index]++;
        }
      }
    }
  }

  // Find the bucket with the most votes.
  size_t idxP = 0, idxT = 0, idxR = 0;
  for (size_t p = 0; p <= kSplitPhi; ++p) {
    for (size_t t = 0; t <= kSplitTheta; ++t) {
      for (size_t r = 0; r <= kSplitR; ++r) {
        if (hough[p][t][r] > hough[idxP][idxT][idxR]) {
          idxP = p;
          idxT = t;
          idxR = r;
        }
      }
    }
  }

  // Find the parameters of that plane.
  const float phi = static_cast<float>(idxP) / kSplitPhi * PI;
  const float theta = static_cast<float>(idxT) / kSplitTheta * (2.0f * PI);
  const float r = (static_cast<float>(idxR) / kSplitR - 0.5f) * 2.0f * maxR;

  // The hough transform produces only a rough estimate, try to refine it
  // using RANSAC.
  {
    // TODO(nand): research & implement this.
    (void) maxIter;
    (void) minPoints;
    (void) eps;
  }

  // Compute the normal vector of the estimated plane.
  const cv::Point3f n(
      std::cos(phi) * std::cos(theta),
      std::cos(phi) * std::sin(theta),
      std::sin(phi));

  // Since the hough transform was performed in a coordinate system whose
  // center was not the origin, we find a point on the plane (n * r) and
  // find out the d coefficient of the plane in the original system using
  // the old normal vector.
  const auto d = (center + n * r).dot(n);

  // The equation of the plane has to be normalized.
  const float l = std::sqrt(n.dot(n) + d * d);

  return { n.x / l, n.y / l, n.z / l, d / l};
}


std::vector<cv::Point3f> transformPlane(
    const std::vector<cv::Point3f> &points,
    const cv::Point3f &n)
{

  // Find the normal vector of the old plane and normalize it.
  const auto ab = points[1] - points[0];
  cv::Point3f planeNormal(0.0f);
  for (const auto &point : points) {
    const auto ac = point - points[0];
    const auto cross = ab.cross(ac);
    const float l = std::sqrt(cross.dot(cross));
    if (l > kEps) {
      planeNormal = cross * (1.0f / l);
      break;
    }
  }
  if (planeNormal.dot(planeNormal) < kEps) {
    throw EXCEPTION() << "Points cannot be collinear.";
  }

  // Normalize the vector of the new plane.
  const float l = std::sqrt(n.dot(n));
  if (l < kEps) {
    throw EXCEPTION() << "Normal vector cannot be null.";
  }

  const cv::Point3f m(n.x / l, n.y / l, n.z / l);

  // Find out the three orthogonal axes of the rotation matrix.
  const auto rotQ = glm::rotation(
      glm::vec3(planeNormal.x, planeNormal.y, planeNormal.z),
      glm::vec3(m.x, m.y, m.z));

  // Find the center point on the old plane - points will be rotated around it.
  const auto &c = findCenter(points);

  // Rotate all points and move them to the origin.
  std::vector<cv::Point3f> newPoints;
  for (const auto &point : points) {
    const auto d = point - c;
    const auto p = glm::rotate(rotQ, {d.x, d.y, d.z, 0.0f});
    newPoints.emplace_back(p.x, p.y, p.z);

  }
  return newPoints;
}

cv::Point3f findCenter(const std::vector<cv::Point3f> &points) {
  const auto sum = std::accumulate(
      points.begin(),
      points.end(),
      cv::Point3f(0.0f));
  return sum * (1.0f / points.size());
}

cv::Point3f findMedianCenter(const std::vector<cv::Point3f> &points) {
  std::vector<float> xList;
  std::vector<float> yList;
  std::vector<float> zList;

  for (const auto &point : points) {
    xList.push_back(point.x);
    yList.push_back(point.y);
    zList.push_back(point.z);
  }

  cv::Point3f centroid;
  centroid.x = findMedian(xList);
  centroid.y = findMedian(yList);
  centroid.z = findMedian(zList);

  return centroid;
}

}

