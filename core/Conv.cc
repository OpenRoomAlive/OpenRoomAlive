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

}}

