// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include "slave/ProCamServer.h"

using namespace dv::slave;


ProCamServer::ProCamServer(
    const std::shared_ptr<libfreenect2::Freenect2Device>& kinect)
  : kinect_(kinect)
{
}

ProCamServer::~ProCamServer() {
}

void ::ProCamServer::getCameraParams(CameraParams& cameraParams) {
  // Retrieve the color camera params from the device.
  libfreenect2::Freenect2Device::ColorCameraParams color
      = kinect_->getColorCameraParams();

  cameraParams.color.fx = color.fx;
  cameraParams.color.fy = color.fy;
  cameraParams.color.cx = color.cx;
  cameraParams.color.cy = color.cy;

  // Retrieve the ir camera params from the device.
  libfreenect2::Freenect2Device::IrCameraParams ir
      = kinect_->getIrCameraParams();

  cameraParams.ir.fx = ir.fx;
  cameraParams.ir.fy = ir.fy;
  cameraParams.ir.cx = ir.cx;
  cameraParams.ir.cy = ir.cy;
  cameraParams.ir.k1 = ir.k1;
  cameraParams.ir.k2 = ir.k2;
  cameraParams.ir.k3 = ir.k3;
  cameraParams.ir.p1 = ir.p1;
  cameraParams.ir.p2 = ir.p2;
}

int32_t ProCamServer::derpderp() {
  printf("DERP DERP\n");
  return -1;
}
