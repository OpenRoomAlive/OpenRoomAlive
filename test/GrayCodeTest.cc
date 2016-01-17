// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <gtest/gtest.h>

#include "core/GrayCode.h"

using namespace dv;


/*
 * Test which checks gray code decoding on a small example: Gray code = 13,
 * binary = 9.
 */
TEST(GreyCode, GreyCodeDecode) {
  uint32_t grayCodeValue = 0b01101;
  uint32_t binaryValue = GrayCode::grayCodeToBinary(grayCodeValue, 5);
  ASSERT_EQ(binaryValue, 0b01001);
}

/*
 * Test which checks encodes a random integer value as a gray code and later
 * decodes it.
 */
TEST(GreyCode, GrayCodeEncode) {
  uint32_t binaryValue = 143851;
  uint32_t grayCodeValue = binaryValue ^ (binaryValue >> 1);
  uint32_t retrievedBinaryValue = GrayCode::grayCodeToBinary(
    grayCodeValue, dv::GrayCode::calculateMaxLevels(binaryValue));
  ASSERT_EQ(binaryValue, retrievedBinaryValue);
}

/*
 * Check of gray code decoding for a corner case: 0.
 */
TEST(GreyCode, GrayCodeDecodeZero) {
  uint32_t binaryValue = 0;
  uint32_t grayCodeValue = binaryValue ^ (binaryValue >> 1);
  uint32_t retrievedBinaryValue = GrayCode::grayCodeToBinary(
    grayCodeValue, dv::GrayCode::calculateMaxLevels(binaryValue));
  ASSERT_EQ(binaryValue, retrievedBinaryValue);
}
