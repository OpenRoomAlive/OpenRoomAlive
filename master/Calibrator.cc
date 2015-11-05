// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <algorithm>
#include <thread>
#include <chrono>
#include <iostream>

#include "core/ProCam.h"
#include "core/GrayCode.h"
#include "master/Calibrator.h"

using namespace dv::master;
using namespace dv;

using namespace std::chrono_literals;

// Duration of a one element of gray code sequence in milliseconds.
constexpr auto kGrayCodeDuration = 2000ms;

Calibrator::Calibrator(
    const std::vector<ConnectionID>& ids,
    const boost::shared_ptr<MasterConnectionHandler>& connectionHandler,
    const std::shared_ptr<ProCamSystem>& system)
  : ids_(ids)
  , connectionHandler_(connectionHandler)
  , system_(system)
{
}

Calibrator::~Calibrator() {
}

void Calibrator::displayGrayCodes() {
  connectionHandler_->clearDisplays();

  for (const auto &id : ids_) {
    auto displayParams = system_->proCams_[id]->getDisplayParams();

    size_t level = slave::GrayCode::calculateLevel(displayParams.frameHeight);
    for (size_t i = 0; i < level; i++) {
      displayAndCapture(id, Orientation::type::HORIZONTAL, i, false);
      displayAndCapture(id, Orientation::type::HORIZONTAL, i, true);
    }

    level = slave::GrayCode::calculateLevel(displayParams.frameWidth);
    for (size_t i = 0; i < level; i++) {
      displayAndCapture(id, Orientation::type::VERTICAL, i, false);
      displayAndCapture(id, Orientation::type::VERTICAL, i, true);
    }

    connectionHandler_->clearDisplay(id);
  }
}

void Calibrator::displayAndCapture(
    ConnectionID id,
    Orientation::type orientation,
    size_t level,
    bool inverted)
{
  // Display interchangebly the vertical and horizontal patterns.
  // Display the vertical uninverted gray code.
  connectionHandler_->displayGrayCode(id, orientation, level, inverted);

  std::this_thread::sleep_for(kGrayCodeDuration);

  // send a command to slave to save the current frame
  auto captured = connectionHandler_->getUndistortedColorImages();
  for (const auto &image : captured) {
    captured_[std::make_pair(id, image.first)].push_back(image.second);
  }
}

Calibrator::GrayCodeMap Calibrator::decode() {
  GrayCodeMap decoded;
  cv::Mat diff, mask, mask16;

  for (const auto &entry : captured_) {
    const auto &images = entry.second;
    auto grayCode = cv::Mat(images[0].size(), CV_16U);

    for (size_t i = 0; i < images.size() / 2; i++) {
      // Compute the difference in the frames and convert to grayscale.
      cv::subtract(images[i * 2], images[i * 2 + 1], diff);
      cv::cvtColor(diff, mask, CV_BGR2GRAY);

      // Threshold the image and convert to 16 bits (needs tweaking).
      cv::threshold(mask, mask, 50 - i * 2, 1, cv::THRESH_BINARY);
      mask.convertTo(mask16, CV_16U);

      // Construct binary code by left shift the current value and add mask.
      cv::scaleAdd(grayCode, 2, mask16, grayCode);
    }
    decoded[entry.first] = grayCode;
  }
  return decoded;
}

void Calibrator::captureBaselines() {
  auto colorBaselines = connectionHandler_->getUndistortedColorImages();
  auto depthBaselines = connectionHandler_->getDepthBaselines();

  for (const auto &id : ids_) {
    auto proCam = system_->getProCam(id);
    proCam->colorBaseline_ = colorBaselines[id];
    proCam->depthBaseline_ = depthBaselines[id];
  }
}
