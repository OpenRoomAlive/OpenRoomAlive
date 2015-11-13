// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <algorithm>
#include <chrono>
#include <iostream>
#include <thread>

#include "core/Geometry.h"
#include "core/GrayCode.h"
#include "core/Projection.h"
#include "core/ProCam.h"
#include "core/Types.h"
#include "master/Calibrator.h"


using namespace dv::master;
using namespace dv;

using namespace std::chrono_literals;


// Duration of a one element of gray code sequence in milliseconds.
constexpr auto kGrayCodeDuration = 300ms;

// Threshold to determine if two color images are the same.
constexpr auto kColorDiffThreshold = 100;

// Depth image resolution -- TODO(ilijar): apply D98 comment.
constexpr size_t kDepthImageWidth = 512;
constexpr size_t kDepthImageHeight = 424;

// Treshold used to filter out some of the depth image noise.
constexpr auto kDepthVarianceTreshold = 10;

// 1m = 1000mm
constexpr float kMilimetersToMeters = 1000.0f;

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

// TODO(ilijar): refactor and get frames in parallel.
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
    auto maxLevel =
      GrayCode::calculateDisplayedLevels(displayParams.frameHeight) - 1;

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

    const size_t horzLevels =
      GrayCode::calculateDisplayedLevels(displayParams.frameHeight);
    for (size_t i = 0; i < horzLevels; i++) {
      displayAndCapture(id, Orientation::type::HORIZONTAL, i, false);
      displayAndCapture(id, Orientation::type::HORIZONTAL, i, true);
    }

    const size_t vertLevels =
      GrayCode::calculateDisplayedLevels(displayParams.frameWidth);
    for (size_t i = 0; i < vertLevels; i++) {
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

Calibrator::GrayCodeBitMaskMap Calibrator::decodeToBitMask() {
  GrayCodeBitMaskMap decoded;
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


void Calibrator::decodeGrayCodes() {
  // Construct the bit mask.
  auto decodedBitMask = decodeToBitMask();
  // Consturct the mappings from color image points to projector points.
  colorToProjPoints(decodedBitMask);
}

void Calibrator::colorToProjPoints(
    Calibrator::GrayCodeBitMaskMap &decodedGrayCodes)
{
  // Iterate over grayCode maps for all pairs of connections.
  for (auto &connectionsCodes : decodedGrayCodes) {

    colorToProj_.emplace(
        connectionsCodes.first, std::vector<std::vector<cv::Point2f>>());

    auto &connectionPair = connectionsCodes.first;
    auto &grayCodes = connectionsCodes.second;

    auto projId = connectionPair.first;
    // Retrieve parameters of the projector.
    auto displayParams = system_->getProCam(projId)->displayParams_;

    // Calculate number of bits needed to encode row and column pixel
    // coordinates.
    size_t rowLevels = GrayCode::calculateDisplayedLevels(
        displayParams.frameHeight);
    size_t colLevels = GrayCode::calculateDisplayedLevels(
        displayParams.frameWidth);

    // Mask for bits encoding row and column coordinates.
    uint32_t rowMask = (1 << rowLevels) - 1;
    uint32_t colMask = (1 << colLevels) - 1;

    for (size_t r = 0; r < static_cast<size_t>(grayCodes.rows); r++) {
      colorToProj_[connectionPair].push_back({});

      for (size_t c = 0; c < static_cast<size_t>(grayCodes.cols); c++) {
        uint32_t encoding = grayCodes.at<uint32_t>(r, c);
        int32_t decRow = -1;
        int32_t decCol = -1;

        // Only greycoded pixels should satisfy this condition, other should
        // have been thresholded.
        //if (encoding > 0 && c > 200 && c < 250 && r > 200 && r < 250) {
        if (encoding > 0) {
          uint32_t rowBits = (encoding >> colLevels) & rowMask;
          uint32_t colBits = encoding & colMask;

          decRow = GrayCode::grayCodeToBinary(rowBits, rowLevels);
          decCol = GrayCode::grayCodeToBinary(colBits, colLevels);
        }

        colorToProj_[connectionPair][r].emplace_back(
            cv::Point2f(decRow, decCol));
      }
    }
  }
}

void Calibrator::calibrate() {
  // KinectID -> [(3D point, 2D Kinect color image)]
  std::unordered_map<
      ConnectionID, std::vector<std::pair<cv::Point3f, cv::Point2f>>>
  kinect3D2D;

  // Find (3D, 2D color image) points captured by the kinects.
  for (const auto &kinectId : ids_) {
    kinect3D2D.emplace(
        kinectId, std::vector<std::pair<cv::Point3f, cv::Point2f>>());

    auto kinect = system_->getProCam(kinectId);
    auto depthImage = kinect->depthBaseline_;

    // Use the depth frame to compute the 3D points.
    for (size_t r = 0; r < kDepthImageHeight; ++r) {
      for (size_t c = 0; c < kDepthImageWidth; ++c) {

        // Extract depth (in meters) and the corresponding variance.
        auto depth = depthImage.at<float>(r, c) / kMilimetersToMeters;
        auto variance = kinect->varianceDepth_.at<float>(r, c);

        // Filter out the noisy points.
        if (equals(depth, 0.0f) || variance > kDepthVarianceTreshold) {
          continue;
        }

        // Compute the 3D point out of the depth info.
        auto point3D = projection::map3D(kinect->irCamMat_, depth, r, c);

        // We're working on undistorted images.
        // Hence, use (r, c) to construct a 2D point in Kinect's color image.
        // Note that the point is wrt the origin in the top left corner.
        auto point2D = cv::Point2f(r, c);

        // Construct the 3D - 2D (color image) correspondence.
        kinect3D2D[kinectId].push_back(std::make_pair(point3D, point2D));
      }
    }
  }

  // Calibrate each projector.
  for (const auto &projectorId : ids_) {
    auto projector = system_->getProCam(projectorId);

    // Vector of vectors of points per each view.
    std::vector<std::vector<cv::Point3f>> worldPointsCalib(ids_.size());
    std::vector<std::vector<cv::Point2f>> projectorPointsCalib(ids_.size());

    // TODO(ilijar): iterate only over the projector group memebers.
    for (size_t i = 0; i < ids_.size(); ++i) {
      const auto &kinectId = ids_[i];
      auto &worldPoints = worldPointsCalib[i];
      auto &projectorPoints = projectorPointsCalib[i];

      // Construct the 3D - 2D (projector) point correspondences.
      for (const auto &pointPair : kinect3D2D[kinectId]) {
        auto kinectColorPoint = pointPair.second;
        auto kinect3DPoint = pointPair.first;

        auto r = kinectColorPoint.x;
        auto c = kinectColorPoint.y;

        // Get the projector point corresponding to the kinect color point.
        auto decodedPoint =
            colorToProj_[std::make_pair(projectorId, kinectId)][r][c];

        // Check if the decoded point is valid.
        if (decodedPoint.x == -1 && decodedPoint.y == -1) {
          continue;
        }

        // TODO(ilijar): Handle resolution/ gray levels -- T51.
        //auto projPoint = cv::Point2f(
        //    768 - decodedPoint.y, 1024 - decodedPoint.x);
        auto projPoint = cv::Point2f(
            64 - decodedPoint.y - 1, 64 - decodedPoint.x - 1);

        // Construct 3D -> 2D correspondance.
        worldPoints.push_back(kinect3DPoint);
        projectorPoints.push_back(projPoint);
      }
    }

    std::vector<std::vector<cv::Point3f>> worldPointsPlane(ids_.size());
    std::vector<std::vector<cv::Point2f>> projectorPointsPlane(ids_.size());

    for (size_t i = 0; i < ids_.size(); ++i) {
      auto &worldPoints = worldPointsPlane[i];
      auto &projectorPoints = projectorPointsPlane[i];

      auto plane = planeFit(worldPointsCalib[i], 200, 100, 0.1f);
      for (size_t j = 0; j < worldPointsCalib[i].size(); ++j) {
        const auto &world = worldPointsCalib[i][j];
        const auto &proj = projectorPointsCalib[i][j];

        const float d =
            plane.nx * world.x +
            plane.ny * world.y +
            plane.nz * world.z -
            plane.d;

        if (std::abs(d) < 0.05f) {
          worldPoints.push_back(world);
          projectorPoints.push_back(proj);
        }
      }
    }

    std::vector<cv::Mat> rvecs, tvecs;

    // TODO(ilijar): make procams use cv::Size for storing resolution
    // TODO(ilijar): same comment as above regarding the resolution/ levels
    auto displayParams = projector->displayParams_;
    cv::Size projectorSize(displayParams.frameWidth, displayParams.frameHeight);

    // Calibrate the projector.
    // TODO(nand): rotate points so all Z's are 0.
    /*auto rms1 = cv::calibrateCamera(
        worldPointsPlane,
        projectorPointsPlane,
        projectorSize,
        projector->projMat_,
        projector->projDist_,
        {},
        {},
        0);*/
    auto rms2 = cv::calibrateCamera(
        worldPointsCalib,
        projectorPointsCalib,
        projectorSize,
        projector->projMat_,
        projector->projDist_,
        rvecs,
        tvecs,
        CV_CALIB_USE_INTRINSIC_GUESS);

    std::cout << "Calibration RMS: " << rms2 << std::endl;
    // TODO(ilijar): remove.
    for (size_t i = 0; i < worldPointsCalib[0].size(); i++) {
      std::cout
          << worldPointsCalib[0][i].x << " "
          << worldPointsCalib[0][i].y << " "
          << worldPointsCalib[0][i].z << " "
          << projectorPointsCalib[0][i].x << " "
          << projectorPointsCalib[0][i].y << " "
          << std::endl;
    }

    std::cout << "projMat: "  << std::endl;
    std::cout << projector->projMat_ << std::endl;

    std::cout << "Rotation vectors: " << std::endl;
    for (const auto &r : rvecs) {
      std::cout << r << " " << std::endl;
    }

    std::cout << "Translation vectors: " << std::endl;
    for (const auto &t : tvecs) {
      std::cout << "Translation vector: " << t << std::endl;
    }
  }
}

