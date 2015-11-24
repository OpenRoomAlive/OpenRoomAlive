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


// Threshold to determine if two color images are the same.
constexpr auto kColorDiffThreshold = 100;

// Depth image resolution -- TODO(ilijar): apply D98 comment.
constexpr size_t kDepthImageWidth = 512;
constexpr size_t kDepthImageHeight = 424;

// Treshold used to filter out some of the depth image noise.
constexpr auto kDepthVarianceTreshold = 10;

// 1m = 1000mm
constexpr float kMilimetersToMeters = 1000.0f;

// Threshold for minimum area of decoded graycode
constexpr auto kDecodedAreaThreshold = 200;

Calibrator::Calibrator(
    const std::vector<ConnectionID>& ids,
    const boost::shared_ptr<ConnectionHandler>& connectionHandler,
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
  auto depthVariances = connectionHandler_->getDepthVariances();

  for (const auto &id : ids_) {
    auto proCam = system_->getProCam(id);
    proCam->colorBaseline_ = colorBaselines[id];
    proCam->depthBaseline_ = depthBaselines[id];
    proCam->depthVariance_ = depthVariances[id];
  }
}

void Calibrator::formProjectorGroups() {
  connectionHandler_->clearDisplays();

  for (const auto &id :ids_) {
    auto proCam = system_->getProCam(id);

    // Project alternative black and white strips
    auto maxLevel = GrayCode::calculateDisplayedLevels(
        proCam->effectiveProjRes_.height) - 1;

    // Capture base images
    connectionHandler_->displayGrayCode(
        id,
        Orientation::type::HORIZONTAL,
        maxLevel,
        false);
    std::this_thread::sleep_for(proCam->getLatency());
    auto base = connectionHandler_->getColorImages();

    // Capture inverted images
    connectionHandler_->displayGrayCode(
        id,
        Orientation::type::HORIZONTAL,
        maxLevel,
        true);
    std::this_thread::sleep_for(proCam->getLatency());
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
    auto effectiveProjRes = system_->proCams_[id]->effectiveProjRes_;

    const size_t horzLevels = GrayCode::calculateDisplayedLevels(
        effectiveProjRes.height);
    const size_t vertLevels = GrayCode::calculateDisplayedLevels(
        effectiveProjRes.width);

    for (size_t i = 0; i < horzLevels; i++) {
      displayAndCapture(id, Orientation::type::HORIZONTAL, i, false);
      displayAndCapture(id, Orientation::type::HORIZONTAL, i, true);
    }

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
  auto proCam = system_->getProCam(id);

  // Display interchangebly the vertical and horizontal patterns.
  // Display the vertical uninverted gray code.
  connectionHandler_->displayGrayCode(id, orientation, level, inverted);

  std::this_thread::sleep_for(proCam->getLatency());

  // send a command to ProCam to save the current frame
  auto captured = connectionHandler_->getColorImages();
  for (const auto &image : captured) {
    captured_[std::make_pair(id, image.first)].push_back(image.second);
  }
}

Calibrator::GrayCodeBitMaskMap Calibrator::decodeToBitMask() {
  GrayCodeBitMaskMap retrievedGraycodes;
  cv::Mat diff, absDiff, mask, mask32;
  cv::Mat grayscale, grayscaleInvert, validPixelsLevel, validPixelsTemp;

  for (const auto &entry : captured_) {
    const auto &images = entry.second;
    cv::Mat validPixelsOverall = cv::Mat::ones(images[0].size(), CV_32S);
    cv::Mat grayCode = cv::Mat::zeros(images[0].size(), CV_32S);

    for (size_t i = 0; i < images.size() / 2; i++) {
      // Convert to grayscales.
      cv::cvtColor(images[i * 2], grayscale, CV_BGR2GRAY);
      cv::cvtColor(images[i * 2 + 1], grayscaleInvert, CV_BGR2GRAY);

      // Find valid pixels in this level
      // TODO1: figure out how to make running it on both coord. work robustly
      // (sometimes improves score a bit, sometimes breaks completely)
      // TODO2: Improve score in general. Gaussian + OTSU?
      //if (i < (images.size() / 4 - 4) ||
      //    (i >= (images.size() / 4) && i < (images.size() / 2 - 5)) ) {
      if (i < (images.size() / 4 - 4)) {
        cv::absdiff(grayscale, grayscaleInvert, absDiff);
        cv::threshold(
            absDiff,
            validPixelsTemp,
            10,
            1,
            cv::THRESH_BINARY);
        validPixelsTemp.convertTo(validPixelsLevel, CV_32S);
      } else {
        validPixelsLevel = cv::Mat::ones(images[0].size(), CV_32S);
      }
      cv::multiply(validPixelsOverall, validPixelsLevel, validPixelsOverall);

      // Retrieve the bitmask from image - compare sets values to 255 in diff
      // where they were bigger in grayscale, then we change them to 1s
      cv::compare(grayscale, grayscaleInvert, diff, cv::CMP_GE);
      cv::threshold(diff, mask, 100, 1, cv::THRESH_BINARY);
      mask.convertTo(mask32, CV_32S);

      // Construct gray code by left shift the current value and add mask.
      cv::scaleAdd(grayCode, 2, mask32, grayCode);
    }

    // Eliminate invalid pixels.
    cv::multiply(grayCode, validPixelsOverall, grayCode);

    cv::Mat grayCodeUndistorted =
        connectionHandler_->undistort(entry.first.second, grayCode);

    removeNoise(grayCodeUndistorted, entry.first.first);
    retrievedGraycodes[entry.first] = grayCodeUndistorted;
  }
  return retrievedGraycodes;
}

void Calibrator::removeNoise(const cv::Mat &grayCode, ConnectionID id) {
  cv::Mat grayCode8;
  grayCode.convertTo(grayCode8, CV_8UC1);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(grayCode8, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

  for (size_t i = 0; i < contours.size(); ++i) {
    auto area = cv::contourArea(contours[i]);
    if (area < kDecodedAreaThreshold) {
      cv::drawContours(grayCode, contours, i, cv::Scalar(0, 0, 0), CV_FILLED);
    } else {
      // Remove reflection
      auto rect = cv::boundingRect(contours[i]);
      auto effectiveProjRes = system_->getProCam(id)->effectiveProjRes_;

      size_t colLevels = GrayCode::calculateDisplayedLevels(
          effectiveProjRes.width);
      size_t rowLevels = GrayCode::calculateDisplayedLevels(
          effectiveProjRes.height);

      uint32_t colMask = (1 << colLevels) - 1;
      uint32_t rowMask = (1 << rowLevels) - 1;

      // Select a horizontal line and check if decoded column is decreasing
      size_t ascending = 0;
      size_t descending = 0;

      for (int y = rect.y; y < rect.y + rect.height; ++y) {
        int gradient = 0;
        for (int x = rect.x; x < rect.x + rect.width - 1; ++x) {
          auto curr = grayCode.at<uint32_t>(y, x);
          auto next = grayCode.at<uint32_t>(y, x + 1);

          // Skip pixels without gray code to account for holes and irregular
          // perimeter of the contour enclosed by bounding rectangle
          if (curr != 0 && next != 0) {
            curr &= colMask;
            next &= colMask;
            auto currCol = GrayCode::grayCodeToBinary(curr, colLevels);
            auto nextCol = GrayCode::grayCodeToBinary(next, colLevels);
            gradient += nextCol - currCol;
          }
        }
        if (gradient < 0) {
          descending++;
        } else if (gradient > 0) {
          ascending++;
        }
      }
      std::cout << "cluster: " << i << " has ascending col: " << ascending
          << " and descending col: " << descending << std::endl;
      if (descending * 2 > ascending) {
        // Discard this cluster
        cv::drawContours(grayCode, contours, i, cv::Scalar(0, 0, 0), CV_FILLED);
      }

      // Select a vertical line and check if decoded row is decreasing
      ascending = 0;
      descending = 0;
      for (int x = rect.x; x < rect.x + rect.width; ++x) {
        int gradient = 0;
        for (int y = rect.y; y < rect.y + rect.height - 1; ++y) {
          auto curr = grayCode.at<uint32_t>(y, x);
          auto next = grayCode.at<uint32_t>(y + 1, x);
          // Skip pixels without gray code to account for holes and irregular
          // perimeter of the contour enclosed by bounding rectangle
          if (curr != 0 && next != 0) {
            curr = (curr >> colLevels) & rowMask;
            next = (next >> colLevels) & rowMask;
            auto currRow = GrayCode::grayCodeToBinary(curr, rowLevels);
            auto nextRow = GrayCode::grayCodeToBinary(next, rowLevels);
            gradient += nextRow - currRow;
          }
        }
        if (gradient < 0) {
          descending++;
        } else if (gradient > 0) {
          ascending++;
        }
      }
      std::cout << "cluster: " << i << " has ascending row: " << ascending
          << " and descending row: " << descending << std::endl;
      if (descending * 2 > ascending) {
        // Discard this cluster
        cv::drawContours(grayCode, contours, i, cv::Scalar(0, 0, 0), CV_FILLED);
      }
    }
  }
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
        connectionsCodes.first,
        std::unordered_map<cv::Point2i, cv::Point2i, cvPoint2iHasher>());

    auto &connectionPair = connectionsCodes.first;
    auto &grayCodes = connectionsCodes.second;

    auto projId = connectionPair.first;
    // Retrieve parameters of the projector.
    auto effectiveProjRes = system_->getProCam(projId)->effectiveProjRes_;

    // Calculate number of bits needed to encode row and column pixel
    // coordinates.
    size_t rowLevels = GrayCode::calculateDisplayedLevels(
        effectiveProjRes.height);
    size_t colLevels = GrayCode::calculateDisplayedLevels(
        effectiveProjRes.width);

    // Mask for bits encoding row and column coordinates.
    uint32_t rowMask = (1 << rowLevels) - 1;
    uint32_t colMask = (1 << colLevels) - 1;

    for (size_t r = 0; r < static_cast<size_t>(grayCodes.rows); r++) {
      for (size_t c = 0; c < static_cast<size_t>(grayCodes.cols); c++) {
        uint32_t encoding = grayCodes.at<uint32_t>(r, c);

        // Only greycoded pixels should satisfy this condition, other should
        // have been thresholded.
        if (encoding > 0) {
          uint32_t rowBits = (encoding >> colLevels) & rowMask;
          uint32_t colBits = encoding & colMask;

          auto decRow = GrayCode::grayCodeToBinary(rowBits, rowLevels);
          auto decCol = GrayCode::grayCodeToBinary(colBits, colLevels);

          colorToProj_[connectionPair].emplace(
              cv::Point2i(r, c),
              cv::Point2i(decRow, decCol));
        }
      }
    }
  }
}

void Calibrator::calibrate() {
  // KinectID -> [(3D point, 2D Kinect color image)]
  std::unordered_map<
      ConnectionID,
      std::vector<std::pair<cv::Point3f, cv::Point2i>>>
  kinect3D2D;

  // Find (3D, 2D color image) points captured by the kinects.
  for (const auto &kinectId : ids_) {
    kinect3D2D.emplace(
        kinectId, std::vector<std::pair<cv::Point3f, cv::Point2i>>());

    auto kinect = system_->getProCam(kinectId);
    auto depthImage = kinect->depthBaseline_;

    // Use the depth frame to compute the 3D points.
    for (size_t r = 0; r < kDepthImageHeight; ++r) {
      for (size_t c = 0; c < kDepthImageWidth; ++c) {

        // Extract depth (in meters) and the corresponding variance.
        auto depth = depthImage.at<float>(r, c) / kMilimetersToMeters;
        auto variance = kinect->depthVariance_.at<float>(r, c);

        // Filter out the noisy points.
        if (equals(depth, 0.0f) || variance > kDepthVarianceTreshold) {
          continue;
        }

        // Compute the 3D point out of the depth info.
        auto point3D = projection::map3D(kinect->irCam_.proj, depth, r, c);

        // We're working on undistorted images.
        // Hence, use (r, c) to construct a 2D point in Kinect's color image.
        // Note that the point is wrt the origin in the top left corner.
        auto point2D = cv::Point2i(r, c);

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
      const auto &colorToProj
          = colorToProj_[std::make_pair(projectorId, kinectId)];

      auto &worldPoints = worldPointsCalib[i];
      auto &projectorPoints = projectorPointsCalib[i];

      // Depending on the projector pixel size, for each 2D point in the
      // projector screen space there may be multiple 3D points mapping to it.
      std::unordered_map<
          cv::Point2i,
          std::vector<cv::Point3f>,
          cvPoint2iHasher>
      buckets;

      // Populate the buckets.
      for (const auto &pointPair : kinect3D2D[kinectId]) {
        auto kinectColorPoint = pointPair.second;
        auto kinect3DPoint = pointPair.first;

        // Get the projector point corresponding to the kinect color point.
        auto decodedPoint = colorToProj.find(kinectColorPoint);

        // Check if the decoded point is valid.
        if (decodedPoint == colorToProj.end()) {
          continue;
        }

        auto bucket = buckets.find(decodedPoint->second);

        // Create a new bucket if needed.
        if (bucket == buckets.end()) {
          buckets.emplace(decodedPoint->second, std::vector<cv::Point3f>());
        }

        buckets[decodedPoint->second].emplace_back(kinect3DPoint);
      }

      // Construct the 3D - 2D (projector) point correspondences.
      for (const auto &bucket : buckets) {
        // Perform the conversion to the projector coordinate system.
        // Origin is in the bottom left corner with +x pointing to the left.
        auto projPoint = cv::Point2f(
            projector->effectiveProjRes_.width - bucket.first.y - 1,
            projector->effectiveProjRes_.height - bucket.first.x - 1);

        // Use centroid of the points from the bucket.
        auto worldPoint = findMedianCenter(bucket.second);

        // Construct 3D -> 2D correspondance.
        worldPoints.push_back(worldPoint);
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

    // Calibrate the projector.
    // TODO(nand): rotate points so all Z's are 0.
    /*auto rms1 = cv::calibrateCamera(
        worldPointsPlane,
        projectorPointsPlane,
        projector->effectiveProjRes_,
        projector->projMat_,
        projector->projDist_,
        {},
        {},
        0);*/

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


    auto rms2 = cv::calibrateCamera(
        worldPointsCalib,
        projectorPointsCalib,
        projector->effectiveProjRes_,
        projector->projMat_,
        projector->projDist_,
        rvecs,
        tvecs,
        CV_CALIB_USE_INTRINSIC_GUESS);

    std::cout << "Calibration RMS: " << rms2 << std::endl;

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

