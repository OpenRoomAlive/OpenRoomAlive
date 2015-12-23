// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <glm/glm.hpp>
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
 * @param depth  Depth value at point (x, y) in meters.
 * @param xx     X value.
 * @param yy     Y value.
 * @return 3D coordinate
 */
cv::Point3f map3D(const cv::Mat& camera, float depth, size_t xx, size_t yy);

/**
 * Converts K, R, T to a view and a projection matrix.
 */
std::pair<glm::mat4, glm::mat4> toProjView(
    const cv::Mat &k,
    const cv::Mat &r,
    const cv::Mat &t);

}}

