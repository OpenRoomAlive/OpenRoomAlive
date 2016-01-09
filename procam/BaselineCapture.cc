// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <algorithm>
#include <iostream>
#include <vector>

#include "procam/BaselineCapture.h"
#include "procam/BGRDCamera.h"

namespace dv { namespace procam {

/// Minimal range of the kinect is 40 cm.
constexpr float kMinDepth = 400.0f;
/// Size of the depth image.
const cv::Size kDepthImageSize(kDepthImageWidth, kDepthImageHeight);


BaselineCapture::BaselineCapture()
  : count_(0)
  , baseline_(kDepthImageSize, kDepthFormat, 0.0f)
  , variance_(kDepthImageSize, kDepthFormat, 0.0f)
  , mask_(kDepthImageSize, CV_8UC1, cv::Scalar(0xFF))
  , ready_(false)
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
  for (size_t r = 0; r < kDepthImageHeight; ++r) {
    for (size_t c = 0; c < kDepthImageWidth; ++c) {
      buf.reserve(kCandidateFrames);

      for (const auto &frame : frames_) {
        const auto depth = frame.at<const float>(r, c);
        if (depth > kMinDepth) {
          buf.push_back(depth);
        }
      }

      // If there are no pixels in more then 10% of frames, the pixel is masked.
      if (buf.size() < kCandidateFrames) {
        mask_.at<uint8_t>(r, c) = 0x00;
      }

      // If there are enough pixels, compute the median and variance.
      // Otherwise discard the pixel and consider it to be noise.
      if (buf.size() < kCandidateFrames / 2) {
        baseline_.at<float>(r, c) = 0.0f;
        variance_.at<float>(r, c) = 0.0f;
        continue;
      }

      // Compute the median.
      std::nth_element(buf.begin(), buf.begin() + buf.size() / 2, buf.end());
      baseline_.at<float>(r, c) = buf[buf.size() / 2];

      // Compute the variance.
      double mean = 0.0;
      for (const auto &d : buf) {
        mean += d;
      }
      mean /= buf.size();

      double var = 0.0;
      for (const auto &d : buf) {
        var += (d - mean) * (d - mean);
      }
      var /= buf.size();

      variance_.at<float>(r, c) = static_cast<float>(var);
      buf.clear();
    }
  }

  // Free all images.
  for (auto &frame : frames_) {
    frame.release();
  }

  // Stop processing.
  {
    std::cout << "Baseline captured." << std::endl;

    std::unique_lock<std::mutex> locker(countLock_);
    ++count_;
    ready_ = true;
    countLock_.unlock();
    countCond_.notify_all();
  }
}

cv::Mat BaselineCapture::getDepthImage() const {
  return baseline_;
}

cv::Mat BaselineCapture::getDepthVariance() const {
  return variance_;
}

cv::Mat BaselineCapture::getDepthMask() const {
  return mask_;
}

void BaselineCapture::framesProcessed() {
  std::unique_lock<std::mutex> locker(countLock_);
  countCond_.wait(locker, [this]() {
    return count_ > kCandidateFrames;
  });
}

}}
