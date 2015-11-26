// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <thread>

#include "core/Conv.h"
#include "procam/LaserDetector.h"

using namespace dv::procam;

using namespace std::literals;

// Window size for laser detection.
constexpr auto kLaserWindowSize = 50;

// Threshold for laser detection.
constexpr auto kLaserThreshold = 30;

// Limit on initial number of potential laser positions.
constexpr auto kMaxPointsLaserAnalysis = 50;

// Width of scaled down frame for processing.
constexpr auto kScaledFrameWidth = 680;

// Height of scaled down frame for processing.
constexpr auto kScaledFrameHeight = 480;

LaserDetector::LaserDetector(
    const std::shared_ptr<Display>& display,
    const std::shared_ptr<MasterClient>& master,
    const std::shared_ptr<BGRDCamera>& camera)
  : display_(display)
  , master_(master)
  , camera_(camera)
{
}

void LaserDetector::detect() {
  // TODO: Send 3D points to master.

  // Frames.
  cv::Mat prev = cv::Mat::zeros(
      cv::Size(kScaledFrameWidth, kScaledFrameHeight),
      CV_8UC1);
  cv::Mat frame, diff, diffThresh;

  // Source of input.
  // TODO: T79 - remove video-based painting once not needed.
  cv::VideoCapture cap("../../video.mov");
  bool useVideo = cap.isOpened();
  if (!useVideo) {
    std::cout << "Using Kinect as input for laser detection." << std::endl;
  }

  // Laser positions.
  bool tracked = false;
  std::shared_ptr<cv::Point2i> track;
  std::vector<std::pair<std::shared_ptr<cv::Point2i>, int32_t>>
      candidates, old, newCandidates;

  while (display_->isRunning()) {
    // Fetch frame from video or Kinect.
    if (useVideo) {
      if (!cap.read(frame)) {
        break;
      }
    } else {
      frame = camera_->getColorImage();
    }

    // Get smaller (HD proc. takes too long) grayscale image.
    cv::resize(frame, frame, cv::Size(kScaledFrameWidth, kScaledFrameHeight));
    cv::cvtColor(frame, frame, CV_BGR2GRAY);

    if (tracked) {
      // If we had a previous position, try to restrict the search area
      // to a 50x50 window around the old position, improving speed
      // and resiliency to noise.
      int32_t offY = std::max(track->y - kLaserWindowSize, 0);
      int32_t height = std::min(
          kScaledFrameHeight - 1,
          track->y + kLaserWindowSize) - offY;
      int32_t offX = std::max(track->x - kLaserWindowSize, 0);
      int32_t width = std::min(
          kScaledFrameWidth - 1,
          track->x + kLaserWindowSize) - offX;
      cv::Mat wndFrame = frame(cv::Rect(offX, offY, width, height));
      cv::Mat wndPrev = prev(cv::Rect(offX, offY, width, height));

      // Compute the difference between two consecutive frames and find
      // the point of maximal intensity in the search window. Noisy points
      // are removed using tresholding since we prefer to loose track and
      // find the pointer again to following noise.
      cv::absdiff(wndFrame, wndPrev, diff);
      cv::threshold(diff, diffThresh, kLaserThreshold, 255, cv::THRESH_TOZERO);
      cv::minMaxLoc(diffThresh, nullptr, nullptr, nullptr, track.get());

      if (track->x > 0 && track->y > 0) {
        track->x += offX;
        track->y += offY;
      } else {
        tracked = false;
      }
    } else {
      // Compute the difference & treshold for noise.
      cv::absdiff(frame, prev, diff);
      cv::threshold(diff, diffThresh, kLaserThreshold, 255, cv::THRESH_TOZERO);

      // Find the points of great intensity.
      double maxv;
      cv::minMaxLoc(diffThresh, nullptr, &maxv, nullptr, nullptr);
      cv::Mat toTrack;
      cv::findNonZero(diffThresh, toTrack);

      if (maxv > kLaserThreshold && toTrack.total() < kMaxPointsLaserAnalysis) {
        newCandidates.clear();
        old.clear();

        // Keep a set of candidate points. If in the new frame we do not
        // have any points close to the old candidates, discard them.
        // If there are neighbours, increase the 'age' of the old points.
        for (const auto &cand : candidates) {
          auto point = cand.first;
          auto age = cand.second;
          int32_t count = 0;
          for (size_t i = 0; i < toTrack.total(); i++) {
            auto newPointTrack = toTrack.at<cv::Point2i>(i);
            cv::Point2i newPoint(newPointTrack.x, newPointTrack.y);
            if (cv::norm(*point - newPoint) < 5){
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
        candidates.clear();
        for (const auto &newCand : newCandidates) {
          candidates.push_back(newCand);
        }
        for (size_t i = 0; i < toTrack.total(); i++) {
          auto newPoint = toTrack.at<cv::Point2i>(i);
          cv::Point2i newPointCopy(newPoint.x, newPoint.y);
          candidates.emplace_back(
              std::make_shared<cv::Point2i>(newPointCopy),
              0);
        }

        // Select the candidate which belongs to the most populous cluster.
        if (!old.empty()) {
          tracked = true;
          std::stable_sort(old.begin(), old.end(),
              [](const std::pair<std::shared_ptr<cv::Point2i>, int32_t> &a,
                 const std::pair<std::shared_ptr<cv::Point2i>, int32_t> &b)
              {
                return a.second >= b.second;
              });
          track = old[0].first;
        }
      }
    }

    // Progress to next frame.
    prev = frame;

    // TODO: detect color and send it + on master side use it (dynamical list
    // of laser colors)
    if (tracked) {
      cv::Scalar color(0, 0, 0);
      dv::Color thriftColor;
      conv::cvScalarToThriftColor(color, thriftColor);
      dv::Point laser;
      laser.x = track->x;
      laser.y = track->y;
      laser.z = 0;
      master_->detectedLaser(laser, thriftColor);
    }

    std::this_thread::sleep_for(10ms);
  }
}