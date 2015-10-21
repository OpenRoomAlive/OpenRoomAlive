// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <iostream>

#include "slave/ProCamServer.h"
#include "slave/RGBDCamera.h"

using namespace dv::slave;


ProCamServer::ProCamServer(const std::shared_ptr<RGBDCamera>& camera)
  : camera_(camera)
{
}

ProCamServer::~ProCamServer() {
}

void ProCamServer::getCameraParams(CameraParams& cameraParams) {
  cameraParams = camera_->getParameters();
}

int32_t ProCamServer::derpderp() {
  std::cout << "DERP DERP" << std::endl;
  return -1;
}
