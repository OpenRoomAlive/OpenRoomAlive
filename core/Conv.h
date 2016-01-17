// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <unordered_map>

#include <libfreenect2/libfreenect2.hpp>
#include <opencv2/opencv.hpp>

#include "core/ProCam.h"
#include "core/Types.h"


/**
 * Contains functions for converting values across types.
 */
namespace dv { namespace conv {


/**
 * Converts a cv::Mat to a thrift frame.
 */
void cvMatToThriftFrame(const cv::Mat &image, Frame &frame);

/**
 * Converts a thrift frame to a cv::Mat.
 */
void thriftFrameToCvMat(const Frame &frame, cv::Mat &image);

/**
 * Converts a map of thrift frames to a map of cv::Mat.
 */
template <typename K>
std::unordered_map<K, cv::Mat>
thriftFrameToCvMatMap(std::unordered_map<K, Frame> frames)
{
  std::unordered_map<K, cv::Mat> images;

  for (const auto& keyFrame : frames) {
    const auto& key = keyFrame.first;
    const auto& frame = keyFrame.second;

    thriftFrameToCvMat(frame, images[key]);
  }

  return images;
}

/**
 * Converts BGRD camera parameters to OpenCV camera and distortion parameters.
 */
CameraModel thriftCamMatToCvMat(
    const BGRCameraParams &bgrParams);

/**
 * Converts depth camera parameters to OpenCV camera and distortion parameters.
 */
CameraModel thriftCamMatToCvMat(
    const IrCameraParams &depthParams);

/**
 * Converts thrift Color to cv::Scalar.
 */
cv::Scalar thriftColorToCvScalar(const Color &color);

/**
 * Converts cv::Scalar to thrift Color.
 */
void cvScalarToThriftColor(const cv::Scalar &color, Color &thriftColor);

/**
 * Converts thrift Point to cv::Point3f.
 */
cv::Point3f thriftPointToCvPoint(const Point &point);

/**
 * Converts cv::Point3f to thrift Point.
 */
void cvPointToThriftPoint(const cv::Point3f &point, Point &thriftPoint);

/**
 * Converts vector of thrift Segment to vector of std::pair<cv::Point2i>.
 */
std::vector<std::pair<cv::Point2i, cv::Point2i>> thriftSegmentsToCvPoints(
    const std::vector<Segment> &segments);

/**
 * Converts vector of std::pair<cv::Point2i> to vector of thrift Segment.
 */
std::vector<Segment> cvPointsToThriftSegments(
    const std::vector<std::pair<cv::Point2i, cv::Point2i>> &path);

/**
 * Converts the color camera parameters in thrift format to libfreenect2 format.
 */
libfreenect2::Freenect2Device::ColorCameraParams thriftToFreenectColorParams(
    const dv::BGRCameraParams &params);

/**
 * Converts the ir camera parameters in thrift format to libfreenect2 format.
 */
libfreenect2::Freenect2Device::IrCameraParams thriftToFreenectIrParams(
    const dv::IrCameraParams &params);

}}
