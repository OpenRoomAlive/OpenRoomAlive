// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <iostream>

#include <opencv2/core/core.hpp>

#include "core/Exception.h"
#include "core/GrayCode.h"

using namespace dv;


/**
 * width and height are clipped off at ceil log2.
 */
GrayCode::GrayCode(size_t width, size_t height)
  : width_(width),
    height_(height)
{
  size_t maxXLevel = calculateMaxLevels(width) - 1;
  size_t maxYLevel = calculateMaxLevels(height) - 1;
  for (size_t x = 0; x <= maxXLevel; x++) {
    verticalBar.emplace_back(generate(width, x, maxXLevel));
  }

  for (size_t y = 0; y <= maxYLevel; y++) {
    horizontalBar.emplace_back(generate(height, y, maxYLevel));
  }
}

std::vector<uint8_t> GrayCode::generate(
    size_t size,
    size_t level,
    size_t maxLevel) {
  // In the formula, level 0 is the one with THINNEST stripes and for us the 
  // level 0 is the first one displayed - with THCICKEST stripes. Hence we use
  // this mask instead of "1 << level".
  auto mask = 1 << (maxLevel - level);
  std::vector<uint8_t> pattern(size);
  for (size_t i = 0; i < size; i++) {
    // compute the levels that the ith pixel should be white
    size_t grayCode = i ^ (i >> 1);
    if (grayCode & mask) {
      // default is 0
      pattern[i] = 255;
    }
  }
  return pattern;
}

GrayCode::~GrayCode() {
}

cv::Mat GrayCode::getPattern(Orientation orientation, size_t level) const {
  switch (orientation) {
    case Orientation::HORIZONTAL:
      if (level < horizontalBar.size()) {
        return cv::Mat(
            static_cast<int>(height_),
            1,
            CV_8UC1,
            const_cast<uint8_t*>(horizontalBar[level].data()));
      }
      throw EXCEPTION()
          << "Level " << level << " out of range ("
          << horizontalBar.size() << ").";
    case Orientation::VERTICAL:
    default:
      if (level < verticalBar.size()) {
        return cv::Mat(
            1,
            static_cast<int>(width_),
            CV_8UC1,
            const_cast<uint8_t*>(verticalBar[level].data()));
      }
      throw EXCEPTION()
          << "Level " << level << " out of range ("
          << verticalBar.size() << ").";
  }
}
