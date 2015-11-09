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
constexpr auto kGrayCodeDuration = 500ms;

// Threshold to determine if two color images are the same.
constexpr auto kColorDiffThreshold = 100;

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

void Calibrator::captureBaselines() {
  auto colorBaselines = connectionHandler_->getColorBaselines();
  auto depthBaselines = connectionHandler_->getDepthBaselines();

  for (const auto &id : ids_) {
    auto proCam = system_->getProCam(id);
    proCam->colorBaseline_ = colorBaselines[id];
    proCam->depthBaseline_ = depthBaselines[id];
  }
}

void Calibrator::formProjectorGroups() {
  connectionHandler_->clearDisplays();

  for (const auto &id :ids_) {

    auto proCam = system_->getProCam(id);
    auto displayParams = proCam->getDisplayParams();

    // Project alternative black and white strips
    auto level = GrayCode::calculateLevel(displayParams.frameHeight) - 1;

    // Capture base images
    connectionHandler_->displayGrayCode(id, Orientation::type::HORIZONTAL, level, false);
    std::this_thread::sleep_for(kGrayCodeDuration);
    auto base = connectionHandler_->getUndistortedColorImages();

    // Capture inverted images
    connectionHandler_->displayGrayCode(id, Orientation::type::HORIZONTAL, level, true);
    std::this_thread::sleep_for(kGrayCodeDuration);
    auto inverted = connectionHandler_->getUndistortedColorImages();

    // Form ProCam group
    for (const auto &target : ids_) {
      // Determine if there's an overlap
      cv::Mat diff;
      cv::absdiff(base[target], inverted[target], diff);

      // Base and inverted images are captured in BGR32 so we convert to grayscale
      cv::cvtColor(diff, diff, CV_BGR2GRAY);
      cv::threshold(diff, diff, 30, 1, cv::THRESH_BINARY);

      //std::cout << "Sum is : " << sum << std::endl;
      if (cv::sum(diff)[0] > kColorDiffThreshold) {
        proCam->projectorGroup_.push_back(target);
        std::cout << "Adding procam: " << target << std::endl;
      }
    }
  }

  connectionHandler_->clearDisplays();
}

void Calibrator::displayGrayCodes() {
  connectionHandler_->clearDisplays();

  for (const auto &id : ids_) {
    auto displayParams = system_->proCams_[id]->getDisplayParams();

    const size_t horzLevel = GrayCode::calculateLevel(displayParams.frameHeight);
    for (size_t i = 0; i < horzLevel; i++) {
      displayAndCapture(id, Orientation::type::HORIZONTAL, i, false);
      displayAndCapture(id, Orientation::type::HORIZONTAL, i, true);
    }

    const size_t vertLevel = GrayCode::calculateLevel(displayParams.frameWidth);
    for (size_t i = 0; i < vertLevel; i++) {
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
  cv::Mat diff, mask, mask32;

  for (const auto &entry : captured_) {
    const auto &images = entry.second;
    cv::Mat grayCode = cv::Mat::zeros(images[0].size(), CV_32S);

    for (size_t i = 0; i < images.size() / 2; i++) {
      // Compute the difference in the frames and convert to grayscale.
      cv::subtract(images[i * 2], images[i * 2 + 1], diff);
      cv::cvtColor(diff, mask, CV_BGR2GRAY);

      // Threshold the image and convert to 32 bits (needs tweaking).
      cv::threshold(mask, mask, 50 - i * 2, 1, cv::THRESH_BINARY);
      mask.convertTo(mask32, CV_32S);

      // Construct binary code by left shift the current value and add mask.
      cv::scaleAdd(grayCode, 2, mask32, grayCode);
    }
    decoded[entry.first] = grayCode;
  }
  return decoded;
}

Calibrator::CapturedPixelsMap Calibrator::grayCodesToPixels(
    Calibrator::GrayCodeMap &decodedGrayCodes)
{
  Calibrator::CapturedPixelsMap pixelsMap;
  // Iterate over grayCode maps for all pairs of connections.
  for (auto &connectionsCodes : decodedGrayCodes) {
    auto &connection = connectionsCodes.first;
    auto &grayCodes = connectionsCodes.second;

    // Retrieve number of rows and columns of the captured images.
    size_t rows = static_cast<size_t>(grayCodes.rows);
    size_t cols = static_cast<size_t>(grayCodes.cols);

    // Calculate number of bits needed to encode row and column pixel
    // coordinates.
    size_t rowLevels = GrayCode::calculateLevel(rows);
    size_t colLevels = GrayCode::calculateLevel(cols);

    // Mask for bits encoding row and column coordinates.
    uint32_t rowMask = (1 << rowLevels) - 1;
    uint32_t colMask = (1 << colLevels) - 1;

    for (size_t i = 0; i < rows; i++) {
      for (size_t j = 0; j < cols; j++) {
        uint32_t encoding = grayCodes.at<uint32_t>(i, j);
        // Only greycoded pixels should satisfy this condition, other should
        // have been thresholded.
        if (encoding > 0) {
          uint32_t rowBits = (encoding >> colLevels) & rowMask;
          uint32_t colBits = encoding & colMask;

          pixelsMap[connection].emplace(std::pair<size_t, size_t>(
              grayCodeToBinary(rowBits, rowLevels),
              grayCodeToBinary(colBits, colLevels)));
        }
      }
    }
  }
  return pixelsMap;
}

