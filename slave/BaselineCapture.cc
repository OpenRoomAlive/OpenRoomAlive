// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <algorithm>
#include <vector>

#include "slave/BaselineCapture.h"
#include "slave/BGRDCamera.h"
#include <iostream>

namespace dv { namespace slave {

/// Max variance tracking threshold.
constexpr float kMinDepth = 0.01f;


BaselineCapture::BaselineCapture()
  : count_(0)
  , baseline_(kDepthImageHeight, kDepthImageWidth, kDepthFormat)
{
}

BaselineCapture::~BaselineCapture() {
}

void BaselineCapture::process(const cv::Mat &frame) {
  // Stop capturing if enough frames were processed.
  if (count_ > kCandidateFrames) {
    return;
  }
  if (count_ < kCandidateFrames) {
    frame.convertTo(frames_[count_++], CV_32FC1);
    return;
  }

  // Traverse the image pixel by pixel.
  std::vector<float> buf;
  for (size_t i = 0; i < kDepthImageHeight; ++i) {
    for (size_t j = 0; j < kDepthImageWidth; ++j) {
      buf.reserve(kCandidateFrames);
      for (const auto &frame : frames_) {
        const auto depth = frame.at<const float>(i, j);
        if (depth > kMinDepth) {
          buf.push_back(depth);
        }
      }

      // If there are enough pixels, compute the median. Otherwise discard
      // the pixel and consider it to be noise.
      if (buf.size() < kCandidateFrames / 2) {
        baseline_.at<float>(i, j) = 0.0f;
      } else {
        std::nth_element(buf.begin(), buf.begin() + buf.size() / 2, buf.end());
        baseline_.at<float>(i, j) = buf[buf.size() / 2];
      }
      buf.clear();
    }
  }

  // Free all images.
  for (auto &frame : frames_) {
    frame.release();
  }

  // Stop processing.
  ++count_;
}

cv::Mat BaselineCapture::getDepthImage() {
  return baseline_;
}

}}
