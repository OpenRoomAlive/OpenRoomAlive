// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <thread>

#include "core/Conv.h"
#include "procam/BaselineCapture.h"
#include "procam/BGRDCamera.h"
#include "procam/LaserDetector.h"

using namespace dv::procam;

using namespace std::literals;

// Window size for laser detection.
constexpr auto kLaserWindowSize = 25;

// Threshold for laser detection.
constexpr auto kLaserThreshold = 30;

// Limit on initial number of potential laser positions.
constexpr auto kMaxPointsLaserAnalysis = 50;

// Width of scaled down frame for processing.
const cv::Size kScaledSize(512, 424);


LaserDetector::LaserDetector(
    const std::shared_ptr<MasterClient>& master,
    const std::shared_ptr<BGRDCamera>& camera,
    const std::shared_ptr<BaselineCapture>& baseline)
  : master_(master)
  , camera_(camera)
  , baseline_(baseline)
  , prev_(cv::Mat::zeros(kScaledSize, CV_8UC1))
  , tracked_(false)
  , scaled_(cv::Mat::zeros(kScaledSize, CV_8UC4))
  , frame_(cv::Mat::zeros(kScaledSize, CV_8UC1))
  , diff_(cv::Mat::zeros(kScaledSize, CV_8UC1))
{
}

void LaserDetector::detect(
    const cv::Mat &rgbFrame,
    const cv::Mat &depth)
{
  (void) depth;

  Candidates old;
  Candidates newCandidates;

  // Scale down (HD proc. takes too long)
  cv::resize(rgbFrame, scaled_, kScaledSize);

  // Extract the red channel.
  cv::extractChannel(scaled_, frame_, 2);

  if (tracked_) {
    cv::Mat diff;

    // If we had a previous position, try to restrict the search area
    // to a 50x50 window around the old position, improving speed
    // and resiliency to noise.
    int32_t offY = std::max(track_.y - kLaserWindowSize, 0);
    int32_t height = std::min(
        kScaledSize.height - 1,
        track_.y + kLaserWindowSize) - offY;
    int32_t offX = std::max(track_.x - kLaserWindowSize, 0);
    int32_t width = std::min(
        kScaledSize.width - 1,
        track_.x + kLaserWindowSize) - offX;
    cv::Mat wndFrame = frame_(cv::Rect(offX, offY, width, height));
    cv::Mat wndPrev = prev_(cv::Rect(offX, offY, width, height));

    // Compute the difference between two consecutive frames and find
    // the point of maximal intensity in the search window. Noisy points
    // are removed using tresholding since we prefer to loose track and
    // find the pointer again to following noise.
    cv::absdiff(wndFrame, wndPrev, diff);
    cv::threshold(diff, diff, kLaserThreshold, 255, cv::THRESH_BINARY);

    // Reset the tracked point.
    track_ = {-100, -100};

    // Find the point closest to the center.
    cv::Mat toTrack;
    cv::findNonZero(diff, toTrack);

    cv::Point2i mid(kLaserWindowSize, kLaserWindowSize);

    if (toTrack.total() >= 1) {
      track_ = toTrack.at<cv::Point2i>(0);
    }

    for (size_t i = 1; i < toTrack.total(); i++) {
      const auto newPoint = toTrack.at<cv::Point2i>(i);
      const float d1 = diff.at<uint8_t>(newPoint) / cv::norm(mid - newPoint);
      const float d2 = diff.at<uint8_t>(track_) / cv::norm(mid - track_);
      if (d1 < d2) {
        track_ = newPoint;
      }
    }

    if (track_.x > 0 && track_.y > 0) {
      track_.x += offX;
      track_.y += offY;
    } else {
      tracked_ = false;
    }
  } else {
    // Compute the difference & treshold for noise.
    cv::absdiff(frame_, prev_, diff_);
    cv::threshold(diff_, diff_, kLaserThreshold, 255, cv::THRESH_TOZERO);

    // Find the points of great intensity.
    double maxv;
    cv::minMaxLoc(diff_, nullptr, &maxv, nullptr, nullptr);
    cv::Mat toTrack;
    cv::findNonZero(diff_, toTrack);

    if (maxv > kLaserThreshold && toTrack.total() < kMaxPointsLaserAnalysis) {
      newCandidates.clear();
      old.clear();

      // Keep a set of candidate points. If in the new frame we do not
      // have any points close to the old candidates, discard them.
      // If there are neighbours, increase the 'age' of the old points.
      for (const auto &cand : candidates_) {
        auto point = cand.first;
        auto age = cand.second;
        int32_t count = 0;
        for (size_t i = 0; i < toTrack.total(); i++) {
          auto newPointTrack = toTrack.at<cv::Point2i>(i);
          cv::Point2i newPoint(newPointTrack.x, newPointTrack.y);
          if (cv::norm(point - newPoint) < 5){
            count += 1;
          }
        }
        if (count > 0) {
          newCandidates.emplace_back(point, age + count);
        }
        if (age + count > 2) {
          old.emplace_back(point, age);
        }
      }

      // Update candidates set.
      candidates_.clear();
      for (const auto &newCand : newCandidates) {
        candidates_.push_back(newCand);
      }
      for (size_t i = 0; i < toTrack.total(); i++) {
        auto newPoint = toTrack.at<cv::Point2i>(i);
        candidates_.emplace_back(newPoint, 0);
      }

      // Select the candidate which belongs to the most populous cluster.
      if (!old.empty()) {
        tracked_ = true;
        std::stable_sort(old.begin(), old.end(),
            [](const std::pair<cv::Point2i, int32_t> &a,
               const std::pair<cv::Point2i, int32_t> &b)
            {
              return a.second >= b.second;
            });
        track_ = old[0].first;
      }
    }
  }

  // Progress to next frame.
  frame_.copyTo(prev_);

  if (tracked_) {
    // Get the laser position in otherwise black HD image.
    cv::Mat laserIm = cv::Mat::zeros(
        kScaledSize,
        kColorFormat);
    laserIm.at<uint32_t>(track_.y, track_.x) = 0xFFFFFFFF;
    cv::resize(
        laserIm,
        laserIm,
        cv::Size(kColorImageWidth, kColorImageHeight));

    // Undistort the HD image and find laser position in depth image.
    track_.x = -1;
    track_.y = -1;
    cv::Mat undistortLaserIm = camera_->undistort(
        laserIm,
        baseline_->getDepthImage());
    for (size_t r = 0; r < kDepthImageHeight; r++) {
      for (size_t c = 0; c < kDepthImageWidth; c++) {
        if (undistortLaserIm.at<uint32_t>(r, c) > 0) {
          track_.x = c;
          track_.y = r;
          break;
        }
      }
    }

    // Send point only if we didn't lose it when undistorting.
    if (track_.x != -1 || track_.y != -1) {
      // Get laser colour.
      dv::Color thriftColor;
      conv::cvScalarToThriftColor({0xFF, 0, 0}, thriftColor);

      // Send depth point corresponding to laser.
      dv::Point laser;
      laser.x = track_.x;
      laser.y = track_.y;
      laser.depth = baseline_->getDepthImage().at<float>(track_.y, track_.x);
      master_->detectedLaser(laser, thriftColor);
    }
  }
}

