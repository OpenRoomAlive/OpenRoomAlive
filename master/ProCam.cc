// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include "ProCam.h"

using namespace dv::master;

ProCam::ProCam()
  : colorCam_({ cv::Mat(3, 3, CV_32F), cv::Mat(0, 0, CV_32F) })
  , irCam_({ cv::Mat(3, 3, CV_32F), cv::Mat(1, 5, CV_32F) })
  , latency_(0)
{
}

ProCam::ProCam(
    const CameraModel &colorCamMat,
    const CameraModel &irCamMat,
    const cv::Size &actualProjRes,
    const cv::Size &effectiveProjRes,
    const std::chrono::milliseconds &latency)
  : colorCam_(colorCamMat)
  , irCam_(irCamMat)
  , actualProjRes_(actualProjRes)
  , effectiveProjRes_(effectiveProjRes)
  , latency_(latency)
{
  projMat_ = cv::Mat::eye(3, 3, cv::DataType<float>::type);

  // Initial guess -- for non planar surfaces.
  projMat_.at<float>(0, 0) = 1000.0f;
  projMat_.at<float>(1, 1) = 1000.0f;
  projMat_.at<float>(0, 2) = effectiveProjRes_.width / 2;
  projMat_.at<float>(1, 2) = 0.0f;

  projDist_ = cv::Mat::zeros(4, 1, cv::DataType<float>::type);
}

ProCam::~ProCam() {
}

