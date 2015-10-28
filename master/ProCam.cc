// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include "ProCam.h"

using namespace dv::master;

ProCam::ProCam(ProCamID id, size_t frameWidth, size_t frameHeight)
  : id_(id)
  , frameWidth_(frameWidth)
  , frameHeight_(frameHeight)
{
}

ProCam::~ProCam() {
}
