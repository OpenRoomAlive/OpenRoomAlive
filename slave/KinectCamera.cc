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
  : freenect_(new libfreenect2::Freenect2())
  , pipeline_(new libfreenect2::OpenGLPacketPipeline())
  , listener_(libfreenect2::Frame::Color | libfreenect2::Frame::Depth)
  , rgb_(kColorImageHeight, kColorImageWidth, kColorFormat)
  , depth_(kDepthImageHeight, kDepthImageWidth, kDepthFormat)
  , rgbUndistorted_(kDepthImageHeight, kDepthImageWidth, kColorFormat)
  , isRunning_(false)
  , frameCount_(0)
{
  // Find the kinect device.
  if (freenect_->enumerateDevices() == 0) {
    throw EXCEPTION() << "No kinect devices found.";
  }
  serial_ = freenect_->getDefaultDeviceSerialNumber();

  // Open it.
  kinect_ = std::shared_ptr<libfreenect2::Freenect2Device>(
      freenect_->openDevice(serial_, pipeline_.get()));

  // Set up the listener.
  kinect_->setColorFrameListener(&listener_);
  kinect_->setIrAndDepthFrameListener(&listener_);

  // Start the kinect.
  kinect_->start();
  isRunning_ = true;

  // Register.
  registration_ = std::make_shared<libfreenect2::Registration>(
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

cv::Mat KinectCamera::getDepthImage() {
  std::lock_guard<std::mutex> locker(framesLock_);
  return depth_;
}

cv::Mat KinectCamera::getRGBImage() {
  std::lock_guard<std::mutex> locker(framesLock_);
  return rgb_;
}

cv::Mat KinectCamera::getUndistortedRGBImage() {
  std::lock_guard<std::mutex> locker(framesLock_);
  return rgbUndistorted_;
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
  libfreenect2::FrameMap frames;
  libfreenect2::Frame undistorted(kDepthImageWidth, kDepthImageHeight, 4);
  libfreenect2::Frame registered(kDepthImageWidth, kDepthImageHeight, 4);

  while (isRunning_) {

    // Wait untill the frames are available.
    listener_.waitForNewFrame(frames);

    // Increment the frame counter.
    {
      std::unique_lock<std::mutex> locker(countLock_);
      frameCount_++;
      countLock_.unlock();
      countCond_.notify_all();
    }

    {
      std::lock_guard<std::mutex> locker(framesLock_);

      // Construct the BGR image.
      auto bgr = cv::Mat(
          frames[libfreenect2::Frame::Color]->height,
          frames[libfreenect2::Frame::Color]->width,
          kColorFormat,
          frames[libfreenect2::Frame::Color]->data).clone();

      // Convert from BGR to RGB.
      cv::cvtColor(bgr, rgb_, CV_BGRA2RGBA);

      // Undistort the image.
      registration_->apply(
          frames[libfreenect2::Frame::Color],
          frames[libfreenect2::Frame::Depth],
          &undistorted,
          &registered);

      // Construct the undistorted depth image.
      depth_ = cv::Mat(
          undistorted.height,
          undistorted.width,
          kDepthFormat,
          undistorted.data).clone();

      // Construct the BGR undistorted image.
      auto bgrUndistorted = cv::Mat(
          registered.height,
          registered.width,
          kColorFormat,
          registered.data).clone();

      // Convert from BGR to RGB.
      cv::cvtColor(bgrUndistorted, rgbUndistorted_, CV_BGR2RGBA);
    }

    listener_.release(frames);
  }
}

void KinectCamera::warmup() {
  std::unique_lock<std::mutex> locker(countLock_);
  countCond_.wait(locker, [this]() {
    return frameCount_ > 0;
  });
}
