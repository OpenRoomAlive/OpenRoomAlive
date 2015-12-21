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
  , projMat_(cv::Mat::eye(3, 3, cv::DataType<float>::type))
  , projDist_(cv::Mat::zeros(4, 1, cv::DataType<float>::type))
{
}

ProCam::~ProCam() {
}

