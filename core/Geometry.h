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
 * @param points    Set of input points.
 * @param maxIter   Maximum number of iterations.
 * @param minPoints Minmum number of points to find on the plane.
 * @param eps       Maximum distance from a point to a plane.
 */
Plane planeFit(
    const std::vector<cv::Point3f> &points,
    unsigned maxIter,
    unsigned minPoints,
    float eps);

}
