// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include "ProCam.h"

using namespace dv::master;

ProCam::ProCam(
    const dv::CameraParams &cameraParams, 
    const dv::DisplayParams &displayParams)
  : cameraParams_(cameraParams)
  , displayParams_(displayParams)
{
}

ProCam::~ProCam() {
}
