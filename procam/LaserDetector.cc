// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <thread>

#include "core/Conv.h"
#include "procam/Display.h"
#include "procam/BGRDCamera.h"
#include "procam/BaselineCapture.h"
#include "procam/LaserDetector.h"

using namespace dv::procam;

/// Size of a colour image.
const cv::Size kColourSize(1920, 1080);
/// Size of a depth image.
const cv::Size kDepthSize(512, 424);
/// Depth difference threshold: 3cm.
constexpr auto kDepthDiffThreshold = 15.0f;
/// Max number of points to ever consider.
constexpr auto kMaxPointsLaserAnalysis = 150;
/// Minimum number of frames required to track a point.
constexpr auto kMinAgeToTrack = 3;
/// Tracking distance.
constexpr auto kTrackDistance = 20.0f;
/// Tracking window size.
const cv::Size kTrackWindowSize(50, 50);

namespace {

/**
 * Returns a window centred around a point.
 */
cv::Rect window(const cv::Point2i &pt, const cv::Size &size) {
  cv::Rect r = {
      pt.x - size.width / 2,
      pt.y - size.height / 2,
      size.width,
      size.height
  };

  r.x      = std::max(0, std::min(r.x, kDepthSize.width - 1));
  r.y      = std::max(0, std::min(r.y, kDepthSize.height - 1));
  r.width  = std::max(0, std::min(r.width, kDepthSize.width - r.x - 1));
  r.height = std::max(0, std::min(r.height, kDepthSize.height - r.y - 1));

  return r;
}

/**
 * Computes the distance between two points.
 */
float dist(const cv::Point2i &a, const cv::Point2i &b) {
  return std::sqrt(
      (a.x - b.x) * (a.x - b.x) +
      (a.y - b.y) * (a.y - b.y)
  );
}

}


LaserDetector::LaserDetector(
    const std::shared_ptr<Display>& display,
    const std::shared_ptr<MasterClient>& master,
    const std::shared_ptr<BGRDCamera> &camera,
    const std::shared_ptr<BaselineCapture>& baseline)
  : display_(display)
  , master_(master)
  , camera_(camera)
  , baseline_(baseline)
  , prev_(kColourSize, CV_16SC1)
  , frame_(kColourSize, CV_8UC1)
  , mask_(kDepthSize, CV_8UC1)
  , laser_(kDepthSize, CV_8UC1)
  , counter_(0)
  , temp(kDepthSize, CV_8UC1)
{
}


void LaserDetector::detect(
    const cv::Mat &frame,
    const cv::Mat &depth)
{
  // Baseline must be computed to proceed.
  if (!baseline_->ready()) {
    return;
  }

  // Increment the frame counter.
  counter_++;

  // Create a depth mask.
  {
    // Threshold on depth difference.
    cv::absdiff(depth, baseline_->getDepthImage(), mask_);
    cv::threshold(
        mask_,
        mask_,
        kDepthDiffThreshold,
        0xFF,
        CV_THRESH_BINARY_INV);
    mask_.convertTo(mask_, CV_8UC1);

    // Combine the masks from this frame & last frame.
    // Combine with the baseline mask.
    cv::bitwise_and(mask_, baseline_->getDepthMask(), mask_);

    // Save the depth for later.
    depth.copyTo(depth_);
  }

  // Convert the current frame to grayscale & compute the difference.
  {
    cv::Mat temp;

    cv::cvtColor(frame, temp, CV_BGR2GRAY);
    cv::GaussianBlur(temp, frame_, {5, 5}, 0);
    frame_.convertTo(frame_, CV_16SC1);

    // Dilate the points of difference A LOT in order to ensure that some
    // of them survive the undistortion step.
    cv::dilate(
        frame_ - prev_,
        diff_,
        cv::getStructuringElement(cv::MORPH_ELLIPSE, {15, 15}, {7, 7})
    );
    cv::threshold(diff_, diff_, 4, 0xFF, CV_THRESH_BINARY);
    diff_.convertTo(diff_, CV_8UC1);
    frame_.copyTo(prev_);
  }

  // Undistort the depth diff image.
  {
    cv::Mat temp;
    cv::cvtColor(diff_, temp, CV_GRAY2BGRA);
    laser_ = camera_->undistort(temp, baseline_->getDepthImage(), CV_8UC4);
    cv::cvtColor(laser_, laser_, CV_BGRA2GRAY);
  }

  // Mask motion in the final image and accumulate neighbours.
  // Also remove regions that are too large (from noise or motion).
  // Small blobs that can represent the pointer are replaced with points.
  {
    cv::Mat temp;
    cv::bitwise_and(laser_, mask_, laser_);

    // Find small regions and drop them.
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(
        laser_,
        contours,
        {},
        CV_RETR_LIST,
        CV_CHAIN_APPROX_SIMPLE,
        {0, 0}
    );

    laser_ = cv::Scalar::all(0);
    for (size_t i = 0; i < contours.size(); ++i) {
      // Drop contours that are less than 5% of an image.
      auto area = cv::contourArea(contours[i]);
      if (area <= 5 || 30 < area) {
        cv::drawContours(laser_, contours, i, {0}, CV_FILLED);
      } else {
        laser_.at<uint8_t>(contours[i][0]) = 0xFF;
      }
    }
  }

  // If the laser pointer was tracked in the last frame, perform the search.
  uint64_t counter = counter_;
  pointers.erase(
      std::remove_if(
          pointers.begin(),
          pointers.end(),
          [counter] (const Pointer &pointer) {
            return pointer.frame + 10 < counter;
          }
      ),
      pointers.end()
  );

  for (auto &pointer : pointers) {
    trackPointer(pointer);
  }

  acquirePointers();
}


void LaserDetector::trackPointer(Pointer &pointer) {
  auto rect = window(pointer.position, kTrackWindowSize);
  auto wnd = laser_(rect);

  // Find the closest point to the original pointer.
  cv::Mat pts;
  cv::Point2i track = {-100, -100};
  const auto &orig = pointer.position - cv::Point2i(rect.x, rect.y);

  cv::findNonZero(wnd, pts);
  for (auto i = pts.begin<cv::Point>(); i != pts.end<cv::Point>(); ++i) {
    const cv::Point2i x((*i).x, (*i).y);

    if (dist(orig, x) < dist(orig, track) && orig != x) {
      track = x;
    }
  }

  if (track.x < 0 || track.y < 0) {
    return;
  }

  track.x += rect.x;
  track.y += rect.y;
  /*
  cv::line(temp, pointer.position, track, {0xFF});
  cv::imshow("temp", temp);
  cv::imshow("laser", laser_);
  */

  pointer.position = track;
  pointer.frame = counter_;
  cv::rectangle(
      laser_,
      window(pointer.position, kTrackWindowSize),
      {0},
      CV_FILLED
  );

  // Send event to master.
  if (master_) {
    dv::Point laser;
    laser.x = track.x;
    laser.y = track.y;
    laser.depth = baseline_->getDepthImage().at<double>(track.y, track.x);

    dv::Color color;
    color.r = 0xFF;
    color.g = 0x00;
    color.b = 0x00;

    master_->detectedLaser(laser, color);
  }
}


void LaserDetector::acquirePointers() {
  // Find all indices where we think the laser pointer might be.
  cv::Mat points;
  cv::findNonZero(laser_, points);

  // If there are too many points, we consider it is due to noise and
  // interrupt tracking until the camera is stabilized.
  if (points.total() >= kMaxPointsLaserAnalysis || points.total() < 1) {
    return;
  }

  // Generate the list of new candidates.
  Candidates newCand;
  for (auto i = points.begin<cv::Point>(); i != points.end<cv::Point>(); ++i) {
    newCand.push_back({cv::Point2i((*i).x, (*i).y), 0, counter_});
  }

  // Remove old candidates that do not have neighbors.
  uint64_t counter = counter_;
  candidates_.erase(
      std::remove_if(
          candidates_.begin(),
          candidates_.end(),
          [counter] (const Candidate &oc) {
            return oc.last + 5 < counter;
          }
      ),
      candidates_.end()
  );

  // Increase age of old candidates.
  for (auto &oc : candidates_) {
    bool increase = false;
    for (const auto &nc : newCand) {
      auto diff = dist(oc.point, nc.point);

      if (diff < kTrackDistance) {
        increase = true;
        break;
      }
    }
    if (increase) {
      oc.last = counter_;
      oc.age++;
    }
  }

  // Append new candidates to the end.
  candidates_.insert(candidates_.end(), newCand.begin(), newCand.end());
  std::stable_sort(
      candidates_.begin(),
      candidates_.end(),
      [] (const Candidate &a, const Candidate &b) {
        return a.last == b.last ? (a.age > b.age) : (a.last > b.last);
      }
  );

  // If the oldest candidate point has reached a certain age, track it.
  if (candidates_[0].age > kMinAgeToTrack) {
    const auto &orig = candidates_[0].point;
    cv::Point2i track = {-100, -100};
    for (const auto &pt : newCand) {
      if (dist(pt.point, orig) < dist(track, orig)) {
        track = pt.point;
      }
    }

    // If point is too close to any of the existing pointers, discard.
    for (const auto &pointer : pointers) {
      if (dist(track, pointer.position) < kTrackDistance) {
        return;
      }
    }

    pointers.emplace_back(track, counter);
    candidates_.clear();
  }
}
