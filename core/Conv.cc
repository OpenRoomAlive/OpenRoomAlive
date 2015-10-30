// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include "core/Conv.h"


namespace dv { namespace conv {


void cvMatToThriftFrame(const cv::Mat& image, Frame& frame) {
  auto data = reinterpret_cast<const char*>(image.data);
  auto size = image.step[0] * image.rows;

  frame.rows = image.rows;
  frame.cols = image.cols;
  frame.format = image.type();
  frame.data = std::string(data, data + size);
}

void thriftFrameToCvMat(Frame& frame, const cv::Mat& image) {
  (void) image;
  (void) frame;

  // TODO(ilijar)
}

}}

