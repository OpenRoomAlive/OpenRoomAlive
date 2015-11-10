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
#include "core/Projection.h"

using namespace dv::master;
using namespace dv;

using namespace std::chrono_literals;

// Duration of a one element of gray code sequence in milliseconds.
constexpr auto kGrayCodeDuration = 300ms;

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

void Calibrator::processDepth() {
  for (const auto &id : ids_) {
    auto proCam = system_->getProCam(id);

    cv::Mat sum = cv::Mat::zeros(424, 512, CV_32FC1);
    cv::Mat sum2 = cv::Mat::zeros(424, 512, CV_32FC1);
    cv::Mat count = cv::Mat::zeros(424, 512, CV_32FC1);

    for (auto i = 0; i < 100; ++i) {
      auto depthImage = connectionHandler_->getDepthImage(id);

      for (auto r = 0; r < 424; ++r) {
        for (auto c = 0; c < 512; ++c) {
          auto depth = depthImage.at<float>(r, c);

          if (depth != 0.0) {
            count.at<float>(r, c) = count.at<float>(r, c) + 1;
            sum.at<float>(r, c) = sum.at<float>(r, c) + depth;
            sum2.at<float>(r, c) = sum2.at<float>(r, c) + depth * depth;
          }
        }
      }
    }

    cv::Mat meanImage = cv::Mat::zeros(424, 512, CV_32FC1);
    cv::Mat varianceImage = cv::Mat::zeros(424, 512, CV_32FC1);

    for (auto r = 0; r < 424; ++r) {
      for (auto c = 0; c < 512; ++c) {
        auto cnt = count.at<float>(r, c);

        if (cnt > 50) {
          auto m = sum.at<float>(r, c) / cnt;
          meanImage.at<float>(r, c) = m;
          varianceImage.at<float>(r, c) = sum2.at<float>(r, c) / cnt - m * m;
        }
      }
    }

    proCam->meanDepth_ = meanImage;
    proCam->varianceDepth_ = varianceImage;
  }
}

void Calibrator::formProjectorGroups() {
  connectionHandler_->clearDisplays();

  for (const auto &id :ids_) {

    auto proCam = system_->getProCam(id);
    auto displayParams = proCam->getDisplayParams();

    // Project alternative black and white strips
    auto maxLevel = std::min(5ul, GrayCode::calculateLevel(displayParams.frameHeight) - 1);

    // Capture base images
    connectionHandler_->displayGrayCode(
        id,
        Orientation::type::HORIZONTAL,
        maxLevel,
        false);
    std::this_thread::sleep_for(kGrayCodeDuration);
    auto base = connectionHandler_->getColorImages();

    // Capture inverted images
    connectionHandler_->displayGrayCode(
        id,
        Orientation::type::HORIZONTAL,
        maxLevel,
        true);
    std::this_thread::sleep_for(kGrayCodeDuration);
    auto inverted = connectionHandler_->getColorImages();

    // Form ProCam group
    for (const auto &target : ids_) {
      // Determine if there's an overlap
      cv::Mat diff;
      cv::absdiff(base[target], inverted[target], diff);

      // Base and inverted images are captured in BGR32 so we convert to grayscale
      cv::cvtColor(diff, diff, CV_BGR2GRAY);
      cv::threshold(diff, diff, 30, 1, cv::THRESH_BINARY);
      if (cv::sum(diff)[0] > kColorDiffThreshold) {
        proCam->projectorGroup_.push_back(target);
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
    Calibrator::GrayCodeMap &decodedGrayCodes,
    ProCamSystem &proCamSys)
{
  Calibrator::CapturedPixelsMap pixelsMap;
  // Iterate over grayCode maps for all pairs of connections.
  for (auto &connectionsCodes : decodedGrayCodes) {
    auto &connectionPair = connectionsCodes.first;
    auto &grayCodes = connectionsCodes.second;

    auto projId = connectionPair.first;
    // Retrieve parameters of the projector.
    auto displayParams = proCamSys.getProCam(projId)->displayParams_;

    // Calculate number of bits needed to encode row and column pixel
    // coordinates.
    size_t rowLevels = GrayCode::calculateLevel(displayParams.frameHeight);
    size_t colLevels = GrayCode::calculateLevel(displayParams.frameWidth);

    // Mask for bits encoding row and column coordinates.
    uint32_t rowMask = (1 << rowLevels) - 1;
    uint32_t colMask = (1 << colLevels) - 1;

    for (size_t i = 0; i < static_cast<size_t>(grayCodes.rows); i++) {
      for (size_t j = 0; j < static_cast<size_t>(grayCodes.cols); j++) {
        uint32_t encoding = grayCodes.at<uint32_t>(i, j);
        // Only greycoded pixels should satisfy this condition, other should
        // have been thresholded.
        if (encoding > 0) {
          uint32_t rowBits = (encoding >> colLevels) & rowMask;
          uint32_t colBits = encoding & colMask;

          const uint32_t encRow = grayCodeToBinary(rowBits, rowLevels);
          const uint32_t encCol = grayCodeToBinary(colBits, colLevels);

          pixelsMap[connectionPair].emplace(
              std::make_pair(encRow, encCol),
              std::make_pair(i, j));
        }
      }
    }
  }
  return pixelsMap;
}

// TODO: T42
Calibrator::CalibrationInput Calibrator::calibrationInput(
      Calibrator::CapturedPixelsMap &capturedPixels,
      std::unordered_map<ConnectionID, cv::Mat> &depthBaseline,
      const std::vector<ConnectionID> &ids,
      ProCamSystem &proCamSys)
{
  Calibrator::CalibrationInput input;

  // Retrieve depth baseline images needed for retrieving 3D coordinates.
  //auto depthBaseline = connectionHandler_->getDepthBaselines();

  for (auto &projId : ids) {
    for (auto &kinectId : ids) {
      auto currKinectPtr = proCamSys.getProCam(kinectId);

      // Get the depth image captured by the kinect camera.
      auto &depthImg = depthBaseline[kinectId];

      auto it = capturedPixels.find(std::make_pair(projId, kinectId));
      // Check if the kinect with kinectID can see the pattern projected by
      // projection with projId, i.e. if they are in the same proCam group.
      if (it != capturedPixels.end()) {
        auto &pixels = it->second;

        std::vector<cv::Point3f> objectPoints;
        std::vector<cv::Point2f> patternPoints;

        for (auto &patternPos : pixels) {
          // Position in the colour image where the gray code pattern was
          // observed (corresponds to the position in the depth image).
          auto pos = patternPos.second;
          // Coordinates of the gray code pixel in the projector space.
          auto pattern = patternPos.first;

          // 3D point in projector space reconstructed from kinect depth image.
          auto point3d = projection::map3D(
              currKinectPtr->irCamMat_,
              depthImg,
              pos.first,
              pos.second);

          // Projection of the 3D point from the projector space to UV space of
          // the kinect.
          // TODO: remove hardcoded value of radial distortion.
          /*auto point2d = projection::project(
              currKinectPtr->colorCamMat_,
              0.18f,
              -0.4f,
              point3d);*/

          // Record the correspondence between the 3D and 2D points.
          if (point3d.z != 0.0f) {
            objectPoints.push_back(point3d);
            patternPoints.emplace_back(pattern.first, pattern.second);
          }
        }
        input.emplace(
            std::make_pair(projId, kinectId),
            std::make_pair(objectPoints, patternPoints));
      }
    }
  }
  return input;
}

Calibrator::CalibrationParams Calibrator::calibrationParams(
    Calibrator::CalibrationInput &input,
    const std::vector<ConnectionID> &ids,
    ProCamSystem &proCamSys)
{
  Calibrator::CalibrationParams params;

  for (auto &projId : ids) {
    auto displayParams = proCamSys.getProCam(projId)->displayParams_;
    // Size of the projected image.
    cv::Size projectorSize(displayParams.frameWidth, displayParams.frameHeight);

    std::vector<ConnectionID> procamGroup;
    // 3D coordinates of the projected patterns in the projector space.
    std::vector<std::vector<cv::Point3f>> objectPoints;
    // Projections of the 3D points onto kinect UV space.
    std::vector<std::vector<cv::Point2f>> patternPoints;

    for (auto &kinectId : ids) {
      auto it = input.find(std::make_pair(projId, kinectId));
      if (it != input.end()) {
        auto &points = it->second;
        // Remember the kinects in the current proCam group.
        procamGroup.emplace_back(kinectId);
        // Use the 3D to 2D point mapping calculated by calibrationInput.
        objectPoints.push_back(points.first);
        patternPoints.push_back(points.second);
      }
    }

    cv::Mat projMat = cv::Mat::eye(3, 3, cv::DataType<double>::type);
    projMat.at<double>(0, 0) = 530.0f;
    projMat.at<double>(1, 1) = 530.0f;
    projMat.at<double>(0, 2) = 315.0f;
    projMat.at<double>(1, 2) = 260.0f;

    cv::Mat distCoeffs = cv::Mat::zeros(4, 1, cv::DataType<double>::type);
    std::vector<cv::Mat> rvecs, tvecs;

    /*for (size_t i=0; i<objectPoints[0].size(); i++) {
      std::cout << objectPoints[0][i].x << " "
                << objectPoints[0][i].y << " "
                << objectPoints[0][i].z << " "
                << patternPoints[0][i].x << " "
                << patternPoints[0][i].y
                << std::endl;
    }*/
    // Calibrate the cameras which can see the pattern displayed by projector
    // with projId.
    double rms = cv::calibrateCamera(
        objectPoints,
        patternPoints,
        projectorSize,
        projMat,
        distCoeffs,
        rvecs,
        tvecs,
        CV_CALIB_USE_INTRINSIC_GUESS);

    std::cout << "Calibration RMS: " << rms << std::endl;

    // Record the output rotation and tranlation vectors.
    for (size_t i = 0; i < procamGroup.size(); i++) {
      params.emplace(
          std::make_pair(projId, procamGroup[i]),
          std::make_pair(rvecs[i], tvecs[i]));
    }
  }
  return params;
}

Calibrator::CalibrationParams Calibrator::calibrate() {
  // TODO: add a guard preventing calling this function before the gray code
  // images are captured.

  auto encodedPixels = decode();

  // Convert gray code encoding to binary.
  auto capturedPixels = grayCodesToPixels(encodedPixels, *system_);

  // Retrieve depth baselines.
  auto depthBaselines = connectionHandler_->getDepthBaselines();

  // Convert the point correspondences to a format suitable for calibration.
  Calibrator::CalibrationInput input = calibrationInput(
      capturedPixels,
      depthBaselines,
      ids_,
      *system_);

  return calibrationParams(input, ids_, *system_);
}
