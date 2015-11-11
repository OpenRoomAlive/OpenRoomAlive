// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <iostream>
#include <memory>
#include <random>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

#include "core/GrayCode.h"
#include "core/ProCam.h"
#include "core/Types.h"
#include "master/Calibrator.h"
#include "master/ProCamSystem.h"

using namespace dv;
using namespace dv::master;

/*
 * Test which checks gray code decoding on an example found on the Internet.
 */
TEST(CalibratorTest, GrayCodeToBinaryConversion1) {
  uint32_t grayCodeValue = 0b01101;
  uint32_t binaryValue = Calibrator::grayCodeToBinary(grayCodeValue, 5);
  ASSERT_EQ(binaryValue, 0b01001);
}

/*
 * Test which checks encodes a random integer value as a gray code and later
 * decodes it.
 */
TEST(CalibratorTest, GrayCodeToBinaryConversion2) {
  uint32_t binaryValue = 143851;
  uint32_t grayCodeValue = binaryValue ^ (binaryValue >> 1);
  uint32_t retrievedBinaryValue = Calibrator::grayCodeToBinary(
    grayCodeValue, dv::GrayCode::calculateMaxLevels(binaryValue));
  ASSERT_EQ(binaryValue, retrievedBinaryValue);
}

/*
 * Check of gray code decoding for a corner case: 0.
 */
TEST(CalibratorTest, GrayCodeToBinaryConversion3) {
  uint32_t binaryValue = 0;
  uint32_t grayCodeValue = binaryValue ^ (binaryValue >> 1);
  uint32_t retrievedBinaryValue = Calibrator::grayCodeToBinary(
    grayCodeValue, dv::GrayCode::calculateMaxLevels(binaryValue));
  ASSERT_EQ(binaryValue, retrievedBinaryValue);
}


class CalibrationTest : public ::testing::Test {
 protected:

  CalibrationTest() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dis(0, 3);

    // IDs of the ProCams present in the system.
    ids_.push_back(1);


    // Parameters of the ProCams.
    cv::Mat colCamParams1(3, 3, CV_32FC1);
    colCamParams1.at<float>(0, 0) = 526.4f;
    colCamParams1.at<float>(1, 1) = 526.4f;
    colCamParams1.at<float>(0, 2) = 313.7f;
    colCamParams1.at<float>(1, 2) = 259.0f;
    colCamParams1.at<float>(2, 2) = 1.0f;

    cv::Mat irCamParams1(3, 3, CV_32FC1);
    irCamParams1.at<float>(0, 0) = 526.4f;
    irCamParams1.at<float>(1, 1) = 526.4f;
    irCamParams1.at<float>(0, 2) = 313.7f;
    irCamParams1.at<float>(1, 2) = 259.0;
    irCamParams1.at<float>(2, 2) = 1.0f;

    cv::Mat irDist1(5, 1, CV_32FC1);
    irDist1.at<float>(0, 0) = 0.1f;
    irDist1.at<float>(1, 0) = 0.1f;
    irDist1.at<float>(2, 0) = 0.1f;
    irDist1.at<float>(3, 0) = 0.1f;
    irDist1.at<float>(4, 0) = 0.1f;

    dv::DisplayParams dispParams1;
    // TODO: fix the tests so that they run with full HD display
    dispParams1.frameWidth = 512; //1920
    dispParams1.frameHeight = 424; //1080

    system_.addProCam(1, colCamParams1, irCamParams1, irDist1, dispParams1);


    // Depth baselines used by procams.
    // First baseline image - plain wall (with some noise added).
    cv::Mat depthImg1(kDepthImageHeight, kDepthImageWidth, CV_32FC1);

    float depth = 1000.0f;
    for (size_t i = 0; i < kDepthImageHeight; i++) {
      for (size_t j = 0; j < kDepthImageWidth; j++) {
        depthImg1.at<float>(i, j) = depth + dis(gen);
      }
    }
    depthBaseline_.emplace(ids_[0], depthImg1);
  }

  // Wrapper around the ProCam system.
  ProCamSystem system_;
  // IDs of the procams in the proCam system.
  std::vector<ConnectionID> ids_;
  // Depth baseline of ProCams.
  std::unordered_map<ConnectionID, cv::Mat> depthBaseline_;
  // Width and height of baseline images.
  const size_t kDepthImageWidth = 512;
  const size_t kDepthImageHeight = 424;
};

/*
 * Check if all the pixels occuring in camera images captured by kinects are
 * correctly decoded.
 */
TEST_F(CalibrationTest, GrayCodesToPixels1) {
  Calibrator::GrayCodeMap capture;
  std::pair<ConnectionID, ConnectionID> connections(1, 1);

  const size_t rows = 424;
  const size_t cols = 512;
  uint32_t grayCodes[rows * cols];

  size_t colLevel = dv::GrayCode::calculateMaxLevels(cols);

  for (size_t i = 0; i < rows; i++) {
    for (size_t j = 0; j < cols; j++) {
      uint32_t rowCode = (i ^ (i >> 1)) << colLevel;
      grayCodes[i * cols + j] = rowCode | (j ^ (j >> 1));
    }
  }

  cv::Mat encodedPixels(rows, cols, CV_32S, &grayCodes);

  capture[connections] = encodedPixels;

  auto decoding = Calibrator::grayCodesToPixels(capture, system_);

  auto &pixelCoords = decoding[connections];

  // Pixel with coordinates 0,0 will not be added to the set as it will get
  // thresholded.
  ASSERT_EQ(pixelCoords.size(), rows * cols - 1);

  for (size_t i = 0; i < rows; i++) {
    for (size_t j = 0; j < cols; j++) {
      if (i != 0 && j != 0) {
        //EXPECT_TRUE(pixelCoords.find(std::pair<size_t, size_t>(i, j)) !=
        //    pixelCoords.end());
      }
    }
  }
}

/*
 * Check that (random) pixel coordinates are correctly encoded.
 */
TEST_F(CalibrationTest, GrayCodesToPixels2) {
  Calibrator::GrayCodeMap capture;
  std::pair<ConnectionID, ConnectionID> connections(1, 1);

  const size_t rows = 424;
  const size_t cols = 512;
  uint32_t grayCodes[rows * cols], rowCode;

  size_t colLevel = dv::GrayCode::calculateMaxLevels(cols);

  for (size_t i = 0; i < rows * cols; i++) {
    grayCodes[i] = 0;
  }

  // Encode pixel (2, 237).
  rowCode = (2 ^ (2 >> 1)) << colLevel;
  grayCodes[2 * cols + 237] = rowCode | (237 ^ (237 >> 1));

  // Encode pixel (332, 420).
  rowCode = (332 ^ (332 >> 1)) << colLevel;
  grayCodes[332 * cols + 420] = rowCode | (420 ^ (420 >> 1));

  cv::Mat encodedPixels(rows, cols, CV_32S, &grayCodes);

  capture[connections] = encodedPixels;

  Calibrator::CapturedPixelsMap decoding =
      Calibrator::grayCodesToPixels(capture, system_);

  auto &pixelCoords = decoding[connections];

  ASSERT_EQ(pixelCoords.size(), 2);
  EXPECT_TRUE(pixelCoords.find(std::pair<size_t, size_t>(2, 237)) !=
      pixelCoords.end());
  EXPECT_TRUE(pixelCoords.find(std::pair<size_t, size_t>(332, 420)) !=
      pixelCoords.end());
}

/*
 * Easy test. We assume a ProCam system made of one ProCam unit. The resolution
 * of the projector is assumed to be the same as the resolution of the depth
 * camera. All pattern pixels are visible by the BGR camra.
 */
TEST_F(CalibrationTest, SingleProCamCalibration) {
  Calibrator::CapturedPixelsMap capturedPixels;

  for (size_t i = 0; i < kDepthImageHeight; i++) {
    for (size_t j = 0; j < kDepthImageWidth; j++) {
      capturedPixels[std::make_pair(1, 1)].emplace(
        std::make_pair(i, j),
        std::make_pair(i, j));
    }
  }

  Calibrator::CalibrationInput input = Calibrator::calibrationInput(
      capturedPixels,
      depthBaseline_,
      ids_,
      system_);

  ASSERT_EQ(input.size(), 1);

  auto connections = input.begin()->first;
  auto pointMappings = input.begin()->second;

  EXPECT_EQ(connections.first, 1);
  EXPECT_EQ(connections.second, 1);

  auto points3D = pointMappings.first;
  auto pointsUV = pointMappings.second;

  ASSERT_EQ(points3D.size(), kDepthImageWidth * kDepthImageHeight);
  ASSERT_EQ(pointsUV.size(), kDepthImageWidth * kDepthImageHeight);

  Calibrator::CalibrationParams params = Calibrator::calibrationParams(
      input,
      ids_,
      system_);

  ASSERT_EQ(params.size(), 1);

  connections = params.begin()->first;
  EXPECT_EQ(connections.first, 1);
  EXPECT_EQ(connections.second, 1);
}
