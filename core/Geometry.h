// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <vector>

#include <opencv2/opencv.hpp>

#include "core/Types.h"


namespace dv {

/**
 * Finds a plane in a set of points using hough transform.
 *
 * @note The equation of the plane will be returned normalized.
 *
 * @param points    Set of input points.
 * @param maxIter   Maximum number of iterations.
 * @param minPoints Minmum number of points to find on the plane.
 * @param eps       Maximum distance from a point to a plane.
 *
 * @return Plane equation.
 */
Plane planeFit(
    const std::vector<cv::Point3f> &points,
    unsigned maxIter,
    unsigned minPoints,
    float eps);

/**
 * Maps all points on a plane to another plane defined by an equation.
 *
 * This is required because apparently in OpenCV 'planar' means 'all points
 * are on a plane parallel to zy, with a variance in z of 1e-5'.
 *
 * @param points   Set of coplanar points.
 * @param n        Plane normal.
 *
 * @return Adjusted points.
 */
std::vector<cv::Point3f> transformPlane(
    const std::vector<cv::Point3f> &points,
    const cv::Point3f &n);

/**
 * Computes the centroid of the set of points.
 *
 * @param points Points whose centroid is to be computed.
 *
 * @return Centroid.
 */
cv::Point3f findCentroid(const std::vector<cv::Point3f> &points);

/**
 * Computes the centroid of the set of points using median instead of mean.
 *
 * @param points Points whose centroid is to be computed.
 *
 * @return Centroid.
 */
cv::Point3f findMedianCenter(const std::vector<cv::Point3f> &points);

}

