// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <vector>

#include <stdint.h>
#include <opencv2/opencv.hpp>

namespace dv {

class GrayCode {
 public:
  GrayCode(size_t width = 1024, size_t height = 1024);
  ~GrayCode();

  enum class Orientation {
    HORIZONTAL,
    VERTICAL
  };

  /**
   * Returns a gray code pattern as a one-dimensional array.
   * Pre: level <= ceil (log2 height)
   */
  cv::Mat getPattern(Orientation orientation, size_t level) const;

  /**
   * Utility function which given a dimension calculates the maximum number of
   * levels that we could display i.e. number of levels needed to precisely
   * encode pixel coordinates.
   */
  static inline size_t calculateMaxLevels(size_t pixels) {
    for (size_t level = 0; level < 32; ++level) {
      if (((size_t) 1 << level) >= pixels) {
        return level;
      }
    }
    return 0;
  }

  /**
   * Returns the number of levels we use when displaying gray codes i.e.
   * calculateMaxLevels adjusted by the fact that images with too thin stripes
   * appear to be all white and we should not use them in calibration.
   */
  static inline size_t calculateDisplayedLevels(size_t pixels) {
    return std::min(6ul, calculateMaxLevels(pixels));
  }

 private:
  size_t width_;
  size_t height_;

  std::vector<std::vector<uint8_t>> horizontalBar;
  std::vector<std::vector<uint8_t>> verticalBar;

  std::vector<uint8_t> generate(size_t size, size_t level, size_t maxLevel);
};

}
