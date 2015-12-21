// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include "core/Compression.h"
#include "core/Conv.h"
#include "core/Exception.h"


namespace dv { namespace conv {


void cvMatToThriftFrame(const cv::Mat &image, Frame &frame) {
  auto data = reinterpret_cast<const char*>(image.data);
  auto size = image.step[0] * image.rows;

  frame.rows = image.rows;
  frame.cols = image.cols;
  frame.format = image.type();
  frame.data = comp::compress(std::string(data, size));
}

void thriftFrameToCvMat(const Frame &frame, cv::Mat &image) {
  image = cv::Mat(
      frame.rows,
      frame.cols,
      frame.format,
      const_cast<char*>(comp::decompress(frame.data).c_str())).clone();
}

/**
 * Converts depth camera parameters to OpenCV camera and distortion parameters.
 */
CameraModel thriftCamMatToCvMat(const BGRCameraParams &bgrParams)
{

  cv::Mat cm(3, 3, cv::DataType<double>::type, cvScalar(0.));

  cm.at<double>(0, 0) = bgrParams.fx;
  cm.at<double>(1, 1) = bgrParams.fy;
  cm.at<double>(0, 2) = bgrParams.cx;
  cm.at<double>(1, 2) = bgrParams.cy;
  cm.at<double>(2, 2) = 1.0;

  return { cm, cv::Mat(0, 0, cv::DataType<double>::type) };
}


/**
 * Converts depth camera parameters to OpenCV camera and distortion parameters.
 */
CameraModel thriftCamMatToCvMat(const IrCameraParams &irParams)
{
  cv::Mat cm(3, 3, cv::DataType<double>::type, cvScalar(0.));

  cm.at<double>(0, 0) = irParams.fx;
  cm.at<double>(1, 1) = irParams.fy;
  cm.at<double>(0, 2) = irParams.cx;
  cm.at<double>(1, 2) = irParams.cy;
  cm.at<double>(2, 2) = 1.0;

  cv::Mat dc(1, 5, cv::DataType<double>::type);

  dc.at<double>(0, 0) = irParams.k1;
  dc.at<double>(0, 1) = irParams.k2;
  dc.at<double>(0, 2) = irParams.p1;
  dc.at<double>(0, 3) = irParams.p2;
  dc.at<double>(0, 4) = irParams.k3;

  return { cm, dc };
}

cv::Scalar thriftColorToCvScalar(const Color &color) {
  return cv::Scalar(color.r, color.g, color.b);
}

void cvScalarToThriftColor(const cv::Scalar &color, Color &thriftColor) {
  thriftColor.r = color[0];
  thriftColor.g = color[1];
  thriftColor.b = color[2];
}

cv::Point3f thriftPointToCvPoint(const Point &point) {
  return cv::Point3f(point.x, point.y, point.depth);
}

std::vector<std::pair<cv::Point2i, cv::Point2i>> thriftSegmentsToCvPoints(
    const std::vector<Segment> &segments)
{
  std::vector<std::pair<cv::Point2i, cv::Point2i>> path;
  for (const auto &seg : segments) {
    path.emplace_back(cv::Point2i(seg.x1, seg.y1), cv::Point2i(seg.x2, seg.y2));
  }
  return path;
}

std::vector<Segment> cvPointsToThriftSegments(
    const std::vector<std::pair<cv::Point2i, cv::Point2i>> &path)
{
  std::vector<Segment> segments;
  for (const auto &segment : path) {
    Segment seg;
    seg.x1 = segment.first.x;
    seg.y1 = segment.first.y;
    seg.x2 = segment.second.x;
    seg.y2 = segment.second.y;
    segments.push_back(seg);
  }
  return segments;
}

}}

