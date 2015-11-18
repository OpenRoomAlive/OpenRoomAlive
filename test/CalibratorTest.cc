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
#include "master/MockConnectionHandler.h"
#include "master/ProCamSystem.h"

using namespace dv;
using namespace dv::master;

/*
 * Test which checks gray code decoding on an example found on the Internet.
 */
TEST(CalibratorTest, GrayCodeToBinaryConversion1) {
  uint32_t grayCodeValue = 0b01101;
  uint32_t binaryValue = GrayCode::grayCodeToBinary(grayCodeValue, 5);
  ASSERT_EQ(binaryValue, 0b01001);
}

/*
 * Test which checks encodes a random integer value as a gray code and later
 * decodes it.
 */
TEST(CalibratorTest, GrayCodeToBinaryConversion2) {
  uint32_t binaryValue = 143851;
  uint32_t grayCodeValue = binaryValue ^ (binaryValue >> 1);
  uint32_t retrievedBinaryValue = GrayCode::grayCodeToBinary(
    grayCodeValue, dv::GrayCode::calculateMaxLevels(binaryValue));
  ASSERT_EQ(binaryValue, retrievedBinaryValue);
}

/*
 * Check of gray code decoding for a corner case: 0.
 */
TEST(CalibratorTest, GrayCodeToBinaryConversion3) {
  uint32_t binaryValue = 0;
  uint32_t grayCodeValue = binaryValue ^ (binaryValue >> 1);
  uint32_t retrievedBinaryValue = GrayCode::grayCodeToBinary(
    grayCodeValue, dv::GrayCode::calculateMaxLevels(binaryValue));
  ASSERT_EQ(binaryValue, retrievedBinaryValue);
}

TEST(MockConnectionHandlerTest, Test1) {
  // Intentionally commented out, test used for manual checking of recorder.
  /*MockConnectionHandler mock("./data/test1");
  cv::Mat image = mock.getDepthImage(0);
  cv::namedWindow("Depth image", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO);
  std::cout << image << std::endl;
  cv::waitKey(0);*/
}
