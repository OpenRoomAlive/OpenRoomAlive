// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include "slave/BaselineCapture.h"
#include "slave/BGRDCamera.h"

namespace dv { namespace slave {

/// Max variance tracking threshold.
constexpr float kMaxVarianceThreshold = 1000.0f;


BaselineCapture::BaselineCapture()
  : count_(0)
  , depth_(kDepthImageHeight, kDepthImageWidth, kDepthFormat)
  , prev_(kDepthImageHeight, kDepthImageWidth, kDepthFormat)
{
}

BaselineCapture::~BaselineCapture() {
}

void BaselineCapture::process(const cv::Mat &frame) {
  // If two consecutive frames are identical, drop the new frame.
  cv::Mat diff;
  cv::absdiff(frame, prev_, diff);
  if (cv::sum(diff)[0] == 0) {
    return;
  }
  frame.copyTo(prev_);
  frame.copyTo(frames_[count_++ % kCandidateFrames]);

  if (count_ < kCandidateFrames) {
    return;
  }
  cv::Mat mask = cv::Mat::zeros(frame.rows, frame.cols, CV_32FC1);
  cv::Mat mean = cv::Mat::zeros(frame.rows, frame.cols, CV_32FC1);

  // Compute a float mask to discard any pixels that contain no information.
  for (const auto &frame : frames_) {
    cv::Mat temp;
    cv::threshold(frame, temp, 0.01f, 1.0f, cv::THRESH_BINARY);
    cv::add(mask, temp, mask);
    cv::add(frame, mean, mean);
  }
  cv::divide(mean, cv::Scalar(kCandidateFrames), mean);

  // Reduce noise in the image. This step aims to maximise the
  // size of contigouos blobs lacking information. If isolated pixels
  // with no information are present, they are not considered as noise.
  cv::erode(
      mask,
      mask,
      cv::getStructuringElement(
          cv::MORPH_ELLIPSE,
          cv::Size(5, 5),
          cv::Point(2, 2)));
  cv::threshold(mask, mask, 1.0f, 1.0f, cv::THRESH_BINARY);

  // Compute the variance in the image batch.
  cv::Mat var = cv::Mat::zeros(frame.rows, frame.cols, CV_32FC1);
  for (const auto &frame : frames_) {
    cv::Mat temp;
    cv::subtract(frame, mean, temp);
    cv::multiply(temp, temp, temp);
    cv::add(temp, var, var);
  }

  cv::multiply(var, mask, var);

  const float noise = std::sqrt(cv::sum(var)[0] / cv::sum(mask)[0]) / kCandidateFrames;
  std::cerr << noise << std::endl;
  (void) kMaxVarianceThreshold;

  cv::Mat tmp;
  cv::multiply(mask, frame, tmp);
  tmp.convertTo(depth_, CV_32FC1);
}

cv::Mat BaselineCapture::getDepthImage() {
  return depth_ / 5000.0f;
}

}}
