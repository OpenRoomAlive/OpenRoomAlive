// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <thread>

#include <libfreenect2/packet_pipeline.h>

#include "core/Exception.h"
#include "slave/KinectCamera.h"

using namespace dv;
using namespace dv::slave;


KinectCamera::KinectCamera()
  : freenect_(new Freenect2())
  , pipeline_(new libfreenect2::OpenGLPacketPipeline())
  , listener_(libfreenect2::Frame::Color | libfreenect2::Frame::Depth)
  , rgb_(kColorImageHeight, kColorImageWidth, kBytesPerPixel)
  , undistorted_(kDepthImageWidth, kDepthImageHeight, kBytesPerPixel)
  , registered_(kDepthImageWidth, kDepthImageHeight, kBytesPerPixel)
  , isRunning_(false)
{
  // Find the kinect device.
  if (freenect_->enumerateDevices() == 0) {
    throw EXCEPTION() << "No kinect devices found.";
  }
  serial_ = freenect_->getDefaultDeviceSerialNumber();

  // Open it.
  kinect_ = std::shared_ptr<Freenect2Device>(
      freenect_->openDevice(serial_, pipeline_.get()));

  // Set up the listener.
  kinect_->setColorFrameListener(&listener_);
  kinect_->setIrAndDepthFrameListener(&listener_);

  // Start the kinect.
  kinect_->start();
  isRunning_ = true;

  // Register.
  registration_ = std::make_shared<Registration>(
      kinect_->getIrCameraParams(), kinect_->getColorCameraParams());

  // Start polling the Kinect.
  dataPolling_ = std::thread([this] () {
    poll();
  });
}

KinectCamera::~KinectCamera() {
  kinect_->stop();
  kinect_->close();
  isRunning_ = false;
  dataPolling_.join();
}

cv::Mat KinectCamera::getDepthFrame() {
  cv::Mat depth;

  {
    std::lock_guard<std::mutex> locker(framesLock_);

    // Consturct a cv::Mat from the frame.
    depth = cv::Mat(
        undistorted_.height,
        undistorted_.width,
        kBytesPerPixel,
        undistorted_.data).clone();
  }

  return depth;
}

cv::Mat KinectCamera::getRGBFrame() {
  std::lock_guard<std::mutex> locker(framesLock_);
  return rgb_;
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

void KinectCamera::poll() {
  FrameMap frames;

  while (isRunning_) {

    // Wait untill the frames are available.
    listener_.waitForNewFrame(frames);

    {
      std::lock_guard<std::mutex> lock(framesLock_);

      // Copy the color image before it gets released.
      rgb_ = cv::Mat(
          frames[Frame::Color]->height,
          frames[Frame::Color]->width,
          kBytesPerPixel,
          frames[Frame::Color]->data).clone();

      // Undistort the image.
      registration_->apply(
          frames[Frame::Color],
          frames[Frame::Depth],
          &undistorted_,
          &registered_);
    }

    listener_.release(frames);
  }
}

