// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include "ProCam.h"

using namespace dv::master;


ProCam::ProCam(
    const cv::Mat &colorCamMat,
    const cv::Mat &irCamMat,
    const cv::Mat &irDist,
    const dv::DisplayParams &displayParams)
  : colorCamMat_(colorCamMat)
  , irCamMat_(irCamMat)
  , irDist_(irDist)
  , displayParams_(displayParams)
{
  projMat_ = cv::Mat::eye(3, 3, cv::DataType<float>::type);

  // Initial guess -- for non planar surfaces.
  projMat_.at<float>(0, 0) = 1000.0f;
  projMat_.at<float>(1, 1) = 1000.0f;
  // TODO(ilijar): T51
  projMat_.at<float>(0, 2) = 32;
  projMat_.at<float>(1, 2) = 0.0f;

  projDist_ = cv::Mat::zeros(4, 1, cv::DataType<float>::type);
}

ProCam::~ProCam() {
}

