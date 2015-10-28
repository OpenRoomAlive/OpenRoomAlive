// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <iostream>

#include <opencv2/core/core.hpp>

#include "core/Exception.h"
#include "core/GrayCode.h"

using namespace dv::slave;


/**
 * width and height are clipped off at ceil log2.
 */
GrayCode::GrayCode(size_t width, size_t height)
  : width_(width),
    height_(height)
{
  size_t numXBits = calculateLevel(width);
  size_t numYBits = calculateLevel(height);

  for (size_t x = 0; x < numXBits; x++) {
    size_t level = numXBits - x - 1;
    verticalBar.emplace_back(generate(width, level));
  }

  for (size_t y = 0; y < numYBits; y++) {
    size_t level = numYBits - y - 1;
    horizontalBar.emplace_back(generate(height, level));
  }
}

std::vector<uint8_t> GrayCode::generate(size_t size, size_t level) {
  std::vector<uint8_t> pattern(size);
  for (size_t i = 0; i < size; i++) {
    // compute the levels that the ith pixel should be white
    size_t grayCode = i ^ (i >> 1);
    if (grayCode & (1 << level)) {
      // default is 0
      pattern[i] = 255;
    }
  }
  return pattern;
}

GrayCode::~GrayCode() {
}

cv::Mat GrayCode::getPattern(Orientation orientation, size_t level) {
  switch (orientation) {
    case Orientation::HORIZONTAL:
      if (level < horizontalBar.size()) {
        return cv::Mat(static_cast<int>(height_), 1, CV_8UC1, horizontalBar[level].data());
      }
      throw EXCEPTION() << "Level " << level << " out of range.";
    case Orientation::VERTICAL:
    default:
      if (level < verticalBar.size()) {
        return cv::Mat(1, static_cast<int>(width_), CV_8UC1, verticalBar[level].data());
      }
      throw EXCEPTION() << "Level " << level << " out of range.";
  }
}
