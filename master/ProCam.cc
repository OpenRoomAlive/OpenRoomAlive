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
}

ProCam::~ProCam() {
}

