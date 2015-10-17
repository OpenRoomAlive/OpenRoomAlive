// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include "core/Exception.h"
#include "slave/KinectCamera.h"

using namespace dv;
using namespace dv::slave;


KinectCamera::KinectCamera()
  : freenect_(new Freenect2())
{
  // Find the kinect device.
  if (freenect_->enumerateDevices() == 0) {
    throw EXCEPTION() << "No kinect devices found.";
  }
  serial_ = freenect_->getDefaultDeviceSerialNumber();

  // Open it.
  kinect_ = std::shared_ptr<Freenect2Device>(freenect_->openDevice(serial_));
  kinect_->start();
}

KinectCamera::~KinectCamera() {
  kinect_->stop();
  kinect_->close();
}

cv::Mat KinectCamera::getRGBFrame() {
  // TODO(ilijar): implement this.
  // 1x1 RGB image.
  return cv::Mat(1, 1, CV_8UC3);
}

cv::Mat KinectCamera::getDepthFrame() {
  // TODO(ilijar): implement this.
  // 1x1 16 bit depth.
  return cv::Mat(1, 1, CV_16UC3);
}

CameraParams KinectCamera::getParameters() {
  CameraParams cameraParams;

  // Retrieve the color camera params from the device.
  auto color = kinect_->getColorCameraParams();
  cameraParams.color.fx = color.fx;
  cameraParams.color.fy = color.fy;
  cameraParams.color.cx = color.cx;
  cameraParams.color.cy = color.cy;

  // Retrieve the ir camera params from the device.
  auto ir = kinect_->getIrCameraParams();
  cameraParams.ir.fx = ir.fx;
  cameraParams.ir.fy = ir.fy;
  cameraParams.ir.cx = ir.cx;
  cameraParams.ir.cy = ir.cy;
  cameraParams.ir.k1 = ir.k1;
  cameraParams.ir.k2 = ir.k2;
  cameraParams.ir.k3 = ir.k3;
  cameraParams.ir.p1 = ir.p1;
  cameraParams.ir.p2 = ir.p2;

  return cameraParams;
}
