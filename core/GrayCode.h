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
   * Utility function which given a dimension calculates the level.
   */
  static inline size_t calculateLevel(size_t pixels) {
    for (size_t level = 0; level < 32; ++level) {
      if (((size_t) 1 << level) >= pixels) {
        return level;
      }
    }
    return 0;
  }

 private:
  size_t width_;
  size_t height_;

  std::vector<std::vector<uint8_t>> horizontalBar;
  std::vector<std::vector<uint8_t>> verticalBar;

  std::vector<uint8_t> generate(size_t size, size_t level);
};

}
