// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <opencv2/opencv.hpp>


/**
 * Provides functions used for camera calibration.
 */
namespace dv { namespace projection {


/**
 * Projects a 3D point to a UV coordinate.
 * Uses procedure outlined in OpenCV Camera Calibration documentation.
 *
 * @param camera Camera matrix of the form [ fx  0 cx]
 *                                         [  0 fy cy]
 *                                         [  0  0  1]
 * @param k1     Radial distortion coefficient
 * @param k2     Radial distortion coefficient
 * @param point  Point in 3D
 * @return UV coordinate
 */
cv::Point2f project(
    const cv::Mat& camera,
    float k1,
    float k2,
    const cv::Point3f& point);

/**
 * Maps the depth image (captured by a procam) to the 3D coordinates
 * of the procam.
 *
 * @param camera Camera matrix of the form [ fx  0 cx]
 *                                         [  0 fy cy]
 *                                         [  0  0  1]
 * @param depth  Depth value at (r, c) in meters.
 * @param r      Row from in which the depth pixel occurs.
 * @param c      Column in which the depth pixel occurs.
 * @return 3D coordinate
 */
cv::Point3f map3D(const cv::Mat& camera, float depth, size_t r, size_t c);

}}

