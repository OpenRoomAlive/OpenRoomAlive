// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <gtest/gtest.h>

#include "core/Conv.h"

using namespace dv;


TEST(ConvTest, CvMatToThriftAndBack) {
  const cv::Mat expected = cv::Mat::eye(3, 3, cv::DataType<float>::type);

  cv::Mat actual;
  Frame thriftFrame;

  conv::cvMatToThriftFrame(expected, thriftFrame);
  conv::thriftFrameToCvMat(thriftFrame, actual);

  cv::Mat diff = expected != actual;

  ASSERT_EQ(cv::countNonZero(diff), 0);
}

TEST(ConvTest, CvScalarToThriftColorAndBack) {
  const cv::Scalar expected(2, 25, 255);

  cv::Scalar actual;
  Color thriftColor;

  conv::cvScalarToThriftColor(expected, thriftColor);
  actual = conv::thriftColorToCvScalar(thriftColor);

  ASSERT_EQ(actual[0], expected[0]);
  ASSERT_EQ(actual[1], expected[1]);
  ASSERT_EQ(actual[2], expected[2]);
}

TEST(ConvTest, CvPointToThriftAndBack) {
  const cv::Point3f expected(4.0f, 2.0f, 3.0f);

  cv::Point3f actual;
  Point thriftPoint;

  conv::cvPointToThriftPoint(expected, thriftPoint);
  actual = conv::thriftPointToCvPoint(thriftPoint);

  ASSERT_EQ(actual.x, expected.x);
  ASSERT_EQ(actual.y, expected.y);
  ASSERT_EQ(actual.z, expected.z);
}

