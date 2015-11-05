// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <gtest/gtest.h>

#include "core/GrayCode.h"
#include "master/Calibrator.h"

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
    grayCodeValue, dv::GrayCode::calculateLevel(binaryValue));
  ASSERT_EQ(binaryValue, retrievedBinaryValue);
}

/*
 * Check of gray code decoding for a corner case: 0.
 */
TEST(CalibratorTest, GrayCodeToBinaryConversion3) {
  uint32_t binaryValue = 0;
  uint32_t grayCodeValue = binaryValue ^ (binaryValue >> 1);
  uint32_t retrievedBinaryValue = Calibrator::grayCodeToBinary(
    grayCodeValue, dv::GrayCode::calculateLevel(binaryValue));
  ASSERT_EQ(binaryValue, retrievedBinaryValue);
}

/*
 * Check if all the pixels occuring in camera images captured by kinects are
 * correctly decoded.
 */
TEST(CalibratorTest, GrayCodesToPixels1) {
  Calibrator::GrayCodeMap capture;
  std::pair<ConnectionID, ConnectionID> connections(1, 1);

  const size_t rows = 424;
  const size_t cols = 512;
  uint32_t grayCodes[rows * cols];

  size_t colLevel = dv::GrayCode::calculateLevel(cols);

  for (size_t i = 0; i < rows; i++) {
    for (size_t j = 0; j < cols; j++) {
      uint32_t rowCode = (i ^ (i >> 1)) << colLevel;
      grayCodes[i * cols + j] = rowCode | (j ^ (j >> 1));
    }
  }

  cv::Mat encodedPixels(rows, cols, CV_32S, &grayCodes);

  capture[connections] = encodedPixels;

  auto decoding = Calibrator::grayCodesToPixels(capture);

  auto &pixelCoords = decoding[connections];

  // Pixel with coordinates 0,0 will not be added to the set as it will get
  // thresholded.
  ASSERT_EQ(pixelCoords.size(), rows * cols - 1);

  for (size_t i = 0; i < rows; i++) {
    for (size_t j = 0; j < cols; j++) {
      if (i != 0 && j != 0) {
        EXPECT_TRUE(pixelCoords.find(std::pair<size_t, size_t>(i, j)) !=
            pixelCoords.end());
      }
    }
  }
}

/*
 * Check that (random) pixel coordinates are correctly encoded.
 */
TEST(CalibratorTest, GrayCodesToPixels2) {
  Calibrator::GrayCodeMap capture;
  std::pair<ConnectionID, ConnectionID> connections(1, 1);

  const size_t rows = 424;
  const size_t cols = 512;
  uint32_t grayCodes[rows * cols], rowCode;

  size_t colLevel = dv::GrayCode::calculateLevel(cols);

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
      Calibrator::grayCodesToPixels(capture);

  auto &pixelCoords = decoding[connections];

  ASSERT_EQ(pixelCoords.size(), 2);
  EXPECT_TRUE(pixelCoords.find(std::pair<size_t, size_t>(2, 237)) !=
      pixelCoords.end());
  EXPECT_TRUE(pixelCoords.find(std::pair<size_t, size_t>(332, 420)) !=
      pixelCoords.end());
}
