// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include "slave/MockDisplay.h"

namespace dv { namespace slave {

MockDisplay::~MockDisplay() {
}

void MockDisplay::run() {
  getchar();
}

void MockDisplay::displayImage(const cv::Mat &image) {
  (void) image;
}

}}
