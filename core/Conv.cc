// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include "core/Conv.h"
#include "core/Exception.h"


namespace dv { namespace conv {


void cvMatToThriftFrame(const cv::Mat& image, Frame& frame) {
  auto data = reinterpret_cast<const char*>(image.data);
  auto size = image.step[0] * image.rows;

  frame.rows = image.rows;
  frame.cols = image.cols;
  frame.format = image.type();
  frame.data = std::string(data, data + size);
}

void thriftFrameToCvMat(const Frame& frame, cv::Mat& image) {
  if (static_cast<size_t>(frame.rows * frame.cols * 4) != frame.data.size()) {
    throw EXCEPTION() << "Invalid frame image.";
  }

  image = cv::Mat(
      frame.rows,
      frame.cols,
      frame.format,
      const_cast<char*>(frame.data.c_str())).clone();
}

cv::Mat thriftCamMatToCvMat(const CameraMatrix& cameraMat) {
  cv::Mat cm(3, 3, cv::DataType<double>::type, cvScalar(0.));

  cm.at<double>(0, 0) = cameraMat.fx;
  cm.at<double>(1, 1) = cameraMat.fy;
  cm.at<double>(0, 2) = cameraMat.cx;
  cm.at<double>(1, 2) = cameraMat.cy;
  cm.at<double>(2, 2) = 1.0;

  return cm;
}

cv::Mat thriftDistToCvMat(const DistCoef& distCoef) {
  cv::Mat dc(1, 5, cv::DataType<double>::type);

  dc.at<double>(0, 0) = distCoef.k1;
  dc.at<double>(0, 1) = distCoef.k2;
  dc.at<double>(0, 2) = distCoef.p1;
  dc.at<double>(0, 3) = distCoef.p2;
  dc.at<double>(0, 4) = distCoef.k3;

  return dc;
}

}}

