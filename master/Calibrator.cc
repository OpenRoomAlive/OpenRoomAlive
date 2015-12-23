// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <algorithm>
#include <chrono>
#include <iostream>
#include <thread>

#include "core/Conv.h"
#include "core/Geometry.h"
#include "core/GrayCode.h"
#include "core/GLViewer.h"
#include "core/Projection.h"
#include "core/ProCam.h"
#include "core/Types.h"
#include "master/Calibrator.h"
#include "master/ConnectionHandler.h"


using namespace dv::master;
using namespace dv;


// Threshold to determine if two color images are the same.
constexpr auto kColorDiffThreshold = 1000;

// Depth image resolution -- TODO(ilijar): apply D98 comment.
constexpr size_t kDepthImageWidth  = 512;
constexpr size_t kDepthImageHeight = 424;
constexpr size_t kColorImageWidth  = 1920;
constexpr size_t kColorImageHeight = 1080;

// Treshold used to filter out some of the depth image noise.
constexpr auto kDepthVarianceTreshold = 36;

// 1m = 1000mm
constexpr float kMilimetersToMeters = 1000.0f;

// Threshold for minimum area of decoded graycode
constexpr auto kDecodedAreaThreshold = 30;

Calibrator::Calibrator(
    const std::vector<ConnectionID>& ids,
    const boost::shared_ptr<ConnectionHandler>& connectionHandler,
    const std::shared_ptr<ProCamSystem>& system)
  : ids_(ids)
  , connectionHandler_(connectionHandler)
  , system_(system)
{
  auto paramsMap = connectionHandler_->getParams();

  for (auto id : ids_) {
    auto params = paramsMap[id];

    registration_[id] = std::shared_ptr<libfreenect2::Registration>(
        new libfreenect2::Registration(
            dv::conv::thriftToFreenectIrParams(params.camera.ir),
            dv::conv::thriftToFreenectColorParams(params.camera.bgr)));
  }
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
        maxLevel / 2,
        false);
    std::this_thread::sleep_for(proCam->getLatency());
    auto base = connectionHandler_->getGrayscaleImages();

    // Capture inverted images
    connectionHandler_->displayGrayCode(
        id,
        Orientation::type::HORIZONTAL,
        maxLevel / 2,
        true);
    std::this_thread::sleep_for(proCam->getLatency());
    auto inverted = connectionHandler_->getGrayscaleImages();

    // Form ProCam group
    for (const auto &target : ids_) {
      // Determine if there's an overlap
      cv::Mat diff;
      cv::absdiff(base[target], inverted[target], diff);

      cv::threshold(diff, diff, 30, 1, cv::THRESH_BINARY);

      if (cv::sum(diff)[0] > kColorDiffThreshold) {
        proCam->projectorGroup_.push_back(target);
      }
    }

    connectionHandler_->clearDisplay(id);
  }

  /*
  // TODO: T80
  std::cout << "Formed projector groups." << std::endl;
  for (const auto projectorId : ids_) {
    std::cout << "Members of projector "
              << projectorId << " group: "
              << std::endl;
    auto projector = system_->getProCam(projectorId);
    for (const auto kinectId : projector->projectorGroup_) {
      std::cout << kinectId << " ";
    }
    std::cout << std::endl << "--------" << std::endl;
  }
  */
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
  auto captured = connectionHandler_->getGrayscaleImages();
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
      cv::GaussianBlur(images[i * 2], grayscale, cv::Size(3, 3), 1);
      cv::GaussianBlur(images[i * 2 + 1], grayscaleInvert, cv::Size(3, 3), 1);

      // Find valid pixels in this level.
      if (i < (images.size() / 4 - 3) ||
         (i >= (images.size() / 4) && i < (images.size() / 2 - 6)) ) {
        cv::absdiff(grayscale, grayscaleInvert, absDiff);
        cv::threshold(
            absDiff,
            validPixelsTemp,
            15,
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
      cv::threshold(diff, mask, 100, 1, cv::THRESH_BINARY | cv::THRESH_OTSU);
      mask.convertTo(mask32, CV_32S);

      // Construct gray code by left shift the current value and add mask.
      cv::scaleAdd(grayCode, 2, mask32, grayCode);
    }

    // Eliminate invalid pixels.
    cv::multiply(grayCode, validPixelsOverall, grayCode);

    cv::Mat grayCodeUndistorted = undistort(grayCode, entry.first.second);

    removeNoise(grayCodeUndistorted);

    retrievedGraycodes[entry.first] = grayCodeUndistorted;
  }
  return retrievedGraycodes;
}


void Calibrator::removeNoise(const cv::Mat &grayCode) {
  cv::Mat grayCode8;
  grayCode.convertTo(grayCode8, CV_8UC1);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(grayCode8, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

  for (size_t i = 0; i < contours.size(); ++i) {
    auto area = cv::contourArea(contours[i]);
    if (area < kDecodedAreaThreshold) {
      cv::drawContours(grayCode, contours, i, cv::Scalar(0, 0, 0), CV_FILLED);
    }
  }
}

void Calibrator::decodeGrayCodes() {
  // Construct the bit mask.
  const auto decodedBitMask = decodeToBitMask();
  // Consturct the mappings from color image points to projector points.
  colorToProjPoints(decodedBitMask);
}

void Calibrator::colorToProjPoints(const GrayCodeBitMaskMap &decodedGrayCodes) {

  // Iterate over grayCode maps for all pairs of connections.
  for (const auto &connectionsCodes : decodedGrayCodes) {
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
      auto encodings = grayCodes.ptr<uint32_t>(r);
      for (size_t c = 0; c < static_cast<size_t>(grayCodes.cols); c++) {
        uint32_t encoding = encodings[c];

        // Only greycoded pixels should satisfy this condition, other should
        // have been thresholded.
        if (encoding <= 0) {
          continue;
        }

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

void Calibrator::calibrate() {
  // KinectID -> [(3D point, 2D Kinect color image)]
  std::unordered_map<
      ConnectionID,
      std::vector<std::pair<cv::Point3f, cv::Point2i>>>
  kinect3D2D;

  // Find (3D, 2D color image) points captured by the kinects.
  for (const auto &kinectId : ids_) {
    auto &map3Dto2D = kinect3D2D[kinectId];
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
        map3Dto2D.push_back(std::make_pair(point3D, point2D));
      }
    }
  }

  // Calibrate each projector.
  for (const auto &projectorId : ids_) {
    std::cout << "Calibrating projector: " << projectorId << std::endl;

    const auto projector = system_->getProCam(projectorId);
    const auto effectiveRes = projector->effectiveProjRes_;
    const auto projectorGroupSize = projector->projectorGroup_.size();

    // Vector of vectors of points per each view.
    std::vector<std::vector<cv::Point3f>> worldPointsCalib(
        projectorGroupSize);
    std::vector<std::vector<cv::Point2f>> projectorPointsCalib(
        projectorGroupSize);

    // Add points from each view (Kinect in the projector group).
    for (size_t i = 0; i < projectorGroupSize; ++i) {
      const auto &kinectId = projector->projectorGroup_[i];
      const auto &colorToProj = colorToProj_[{projectorId, kinectId}];

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
        auto &kinectColorPoint = pointPair.second;
        auto &kinect3DPoint = pointPair.first;

        // Get the projector point corresponding to the kinect color point.
        auto decodedPoint = colorToProj.find(kinectColorPoint);

        // Check if the decoded point is valid.
        if (decodedPoint == colorToProj.end()) {
          continue;
        }

        // Add the point to the bucket.
        buckets[decodedPoint->second].emplace_back(kinect3DPoint);
      }

      // Construct the 3D - 2D (projector) point correspondences.
      for (const auto &bucket : buckets) {
        // Perform the conversion to the projector coordinate system.
        // Origin is in the bottom left corner with +x pointing to the left.
        auto projPoint = cv::Point2f(
            effectiveRes.width - bucket.first.y - 1,
            effectiveRes.height - bucket.first.x - 1);

        // Use centroid of the points from the bucket.
        auto worldPoint = findMedianCenter(bucket.second);

        // Construct 3D -> 2D correspondance.
        worldPoints.push_back(worldPoint);
        projectorPoints.push_back(projPoint);
      }
    }

    /*
    // Try to avoid using initial intrinsic guess.
    std::vector<std::vector<cv::Point3f>> worldPointsPlane(
        projectorGroupSize);
    std::vector<std::vector<cv::Point2f>> projectorPointsPlane(
        projectorGroupSize);

    for (size_t i = 0; i < projectorGroupSize; ++i) {
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

      // Transform the points so they are on a plane with z = 0.
      worldPoints = transformPlane(worldPoints, {0, 0, 1});
      for (auto &pt : worldPoints) {
        pt.z = 0.0f;
      }
    }

    // Find the calibration matrix in two steps.
    // First, a set of points on a planar surface with z = 0 is used to
    // compute an initial guess for the camera parameters, after which
    // the projection matrix is refined using all the correspondences.
    projector->projMat_ = cv::initCameraMatrix2D(
        worldPointsPlane,
        projectorPointsPlane,
        projector->effectiveProjRes_,
        static_cast<float>(effectiveRes.width) /
        static_cast<float>(effectiveRes.height)
    );

    auto rms = cv::calibrateCamera(
       worldPointsCalib,
        projectorPointsCalib,
        projector->effectiveProjRes_,
        projector->projMat_,
        projector->projDist_,
        {},
        {},
        CV_CALIB_USE_INTRINSIC_GUESS |
        CV_CALIB_FIX_PRINCIPAL_POINT
    );
    std::cerr
        << projector->projMat_ << std::endl
        << projector->projDist_ << std::endl;

    // Store the computed rotation and translation vectors.
    for (size_t i = 0; i < ids_.size(); ++i) {
      cv::solvePnP(
          worldPointsCalib[i],
          projectorPointsCalib[i],
          projector->projMat_,
          projector->projDist_,
          projector->poses[i].rvec,
          projector->poses[i].tvec,
          false,
          CV_ITERATIVE);
   }

    std::cout
        << "Projector #" << projectorId << std::endl
        << "  RMS: " << rms << std::endl
        << "  matrix: " << projector->projMat_ << std::endl;

    core::GLViewer viewer([&] {
      viewer.drawPoints(
          worldPointsCalib[0],
          projectorPointsCalib[0],
          projector->effectiveProjRes_);
      viewer.drawCamera(
          projector->projMat_,
          projector->poses[0].rvec,
          projector->poses[0].tvec);
    });
    */

    // If you are running the calibration using initCameraMatrix2D & solvePnP,
    // don't delete the following. Comment it out as it is done above.
    projector->projMat_.at<float>(0, 0) = 1000.0f;
    projector->projMat_.at<float>(1, 1) = 1000.0f;
    projector->projMat_.at<float>(0, 2) = effectiveRes.width / 2;
    projector->projMat_.at<float>(1, 2) = 0.0f;
    projector->projMat_.at<float>(2, 2) = 1.0f;

    std::vector<cv::Mat> rvecs, tvecs;

    auto rms = cv::calibrateCamera(
        worldPointsCalib,
        projectorPointsCalib,
        effectiveRes,
        projector->projMat_,
        projector->projDist_,
        rvecs,
        tvecs,
        CV_CALIB_USE_INTRINSIC_GUESS);

    // Store the computed rotation and translation vectors.
    for (size_t i = 0; i < projector->projectorGroup_.size(); ++i) {
      CameraPose p;
      p.rvec = rvecs[i];
      p.tvec = tvecs[i];
      projector->poses[projector->projectorGroup_[i]] = p;
    }

    std::cout << "Projector #" << projectorId << std::endl;
    std::cout << "Points used: " << worldPointsCalib[0].size() << std::endl;
    std::cout << "RMS: " << rms << std::endl;
    std::cout << "Calibration matrix: " << std::endl;
    std::cout << projector->projMat_ << std::endl;
  }

  // TODO(ilijar): remove.
  std::cout << "Rotation and translation vectors:" << std::endl;
  for (auto projectorId : ids_) {
    std::cout << "Projector: " << projectorId << std::endl;
    auto projector = system_->getProCam(projectorId);
    for (auto kinectId : projector->projectorGroup_) {
      auto pose = projector->poses[kinectId];
      std::cout
          << "Kinect: " << kinectId << std::endl
          << "Tvec: " << std::endl << pose.tvec << std::endl
          << "Rvec: " << std::endl << pose.rvec << std::endl
          << "--------------------" << std::endl;
    }
  }
}

cv::Mat Calibrator::undistort(
    const cv::Mat &HDImage,
    ConnectionID id)
{
  // Retrieve the registration object for the current proCam.
  std::shared_ptr<libfreenect2::Registration> registration = registration_[id];

  // Retrieve the depth baseline for image undistortion.
  const cv::Mat &depthImage = system_->getProCam(id)->depthBaseline_;

  // Prepare HD colour and depth frames.
  libfreenect2::Frame HDFrame(kColorImageWidth, kColorImageHeight, 4);
  libfreenect2::Frame depthFrame(kDepthImageWidth, kDepthImageHeight, 4);
  HDFrame.data = HDImage.data;
  depthFrame.data = depthImage.data;

  libfreenect2::Frame depthUndistortedFrame(
      kDepthImageWidth,
      kDepthImageHeight,
      4);
  libfreenect2::Frame HDundistortedFrame(
      kDepthImageWidth,
      kDepthImageHeight,
      4);

  // Undistort images.
  registration->apply(
      &HDFrame,
      &depthFrame,
      &depthUndistortedFrame,
      &HDundistortedFrame);

  // Construct the BGR graycode undistorted image.
  cv::Mat undistorted;
  cv::Mat(
      HDundistortedFrame.height,
      HDundistortedFrame.width,
      CV_32S,
      HDundistortedFrame.data).copyTo(undistorted);

  return undistorted;
}
