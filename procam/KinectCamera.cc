// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <thread>

#include <libfreenect2/packet_pipeline.h>

#include "core/Exception.h"
#include "procam/KinectCamera.h"

using namespace dv;
using namespace dv::procam;

KinectCamera::KinectCamera(
    uint16_t logLevel,
    const std::string &logFilename)
  : freenect_(new libfreenect2::Freenect2())
  , pipeline_(new libfreenect2::OpenGLPacketPipeline())
  , kinect_(nullptr)
  , listener_(libfreenect2::Frame::Color | libfreenect2::Frame::Depth)
  , registration_(nullptr)
  , logger_(
        new KinectFileLogger(
            static_cast<KinectFileLogger::Level>(logLevel),
            logFilename),
        [] (KinectFileLogger*) {
          libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(
              libfreenect2::Logger::getDefaultLevel()));
        })
  , bgr_(kColorImageHeight, kColorImageWidth, kColorFormat)
  , depth_(kDepthImageHeight, kDepthImageWidth, kDepthFormat)
  , bgrUndistorted_(kDepthImageHeight, kDepthImageWidth, kColorFormat)
  , isRunning_(false)
  , frameCount_(0)
{

  // Prepare a custom logger for Kinect messages.
  auto level = static_cast<KinectFileLogger::Level>(logLevel);
  auto levelStr = libfreenect2::Logger::level2str(level);
  libfreenect2::setGlobalLogger(logger_.get());
  std::cout
      << "Logging to \"" << logFilename << "\" at level " << logLevel << " - "
      << levelStr << "." << std::endl;
  logger_->log(level, "<- This log contains msgs up to this importance level.");

  // Find the kinect device.
  if (freenect_->enumerateDevices() == 0) {
    throw EXCEPTION() << "No kinect devices found.";
  }
  serial_ = freenect_->getDefaultDeviceSerialNumber();

  // Open it.
  if ((kinect_ = freenect_->openDevice(serial_, pipeline_)) == nullptr) {
    throw EXCEPTION() << "Cannot connect to kinect device.";
  }

  // Set up the listener.
  kinect_->setColorFrameListener(&listener_);
  kinect_->setIrAndDepthFrameListener(&listener_);

  // Start the kinect.
  kinect_->start();
  isRunning_ = true;

  // Register.
  registration_ = new libfreenect2::Registration(
      kinect_->getIrCameraParams(),
      kinect_->getColorCameraParams());

  // Start polling the Kinect.
  dataPolling_ = std::thread([this] () {
    poll();
  });
}

KinectCamera::~KinectCamera() {
  kinect_->stop();
  kinect_->close();

  // Make sure to free stuff in order.
  if (registration_) {
    delete registration_;
  }
  if (kinect_) {
    delete kinect_;
  }
  if (pipeline_) {
    delete pipeline_;
  }
  if (freenect_) {
    delete freenect_;
  }

  // Stop the kinect thread.
  isRunning_ = false;
  dataPolling_.join();
}

cv::Mat KinectCamera::getDepthImage() {
  std::lock_guard<std::mutex> locker(framesLock_);
  return depth_;
}

cv::Mat KinectCamera::getColorImage() {
  std::lock_guard<std::mutex> locker(framesLock_);
  return bgr_;
}

cv::Mat KinectCamera::getUndistortedColorImage() {
  std::lock_guard<std::mutex> locker(framesLock_);
  return bgrUndistorted_;
}

CameraParams KinectCamera::getParameters() {
  CameraParams cameraParams;

  // Retrieve the color camera params from the device.
  {
    auto color = kinect_->getColorCameraParams();
    cameraParams.bgr.fx = color.fx;
    cameraParams.bgr.fy = color.fy;
    cameraParams.bgr.cx = color.cx;
    cameraParams.bgr.cy = color.cy;

    cameraParams.bgr.shift_d = color.shift_d;
    cameraParams.bgr.shift_m = color.shift_m;

    cameraParams.bgr.mx_x3y0 = color.mx_x3y0;
    cameraParams.bgr.mx_x0y3 = color.mx_x0y3;
    cameraParams.bgr.mx_x2y1 = color.mx_x2y1;
    cameraParams.bgr.mx_x1y2 = color.mx_x1y2;
    cameraParams.bgr.mx_x2y0 = color.mx_x2y0;
    cameraParams.bgr.mx_x0y2 = color.mx_x0y2;
    cameraParams.bgr.mx_x1y1 = color.mx_x1y1;
    cameraParams.bgr.mx_x1y0 = color.mx_x1y0;
    cameraParams.bgr.mx_x0y1 = color.mx_x0y1;
    cameraParams.bgr.mx_x0y0 = color.mx_x0y0;
    cameraParams.bgr.my_x3y0 = color.my_x3y0;
    cameraParams.bgr.my_x0y3 = color.my_x0y3;
    cameraParams.bgr.my_x2y1 = color.my_x2y1;
    cameraParams.bgr.my_x1y2 = color.my_x1y2;
    cameraParams.bgr.my_x2y0 = color.my_x2y0;
    cameraParams.bgr.my_x0y2 = color.my_x0y2;
    cameraParams.bgr.my_x1y1 = color.my_x1y1;
    cameraParams.bgr.my_x1y0 = color.my_x1y0;
    cameraParams.bgr.my_x0y1 = color.my_x0y1;
    cameraParams.bgr.my_x0y0 = color.my_x0y0;
  }

  // Retrieve the ir camera params from the device.
  {
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
  }

  return cameraParams;
}

void KinectCamera::poll() {
  libfreenect2::FrameMap frames;
  libfreenect2::Frame undistorted(kDepthImageWidth, kDepthImageHeight, 4);
  libfreenect2::Frame registered(kDepthImageWidth, kDepthImageHeight, 4);

  while (isRunning_) {

    // Wait untill the frames are available.
    listener_.waitForNewFrame(frames);

    {
      std::lock_guard<std::mutex> locker(framesLock_);

      // Construct the BGR image.
      cv::Mat(
          frames[libfreenect2::Frame::Color]->height,
          frames[libfreenect2::Frame::Color]->width,
          kColorFormat,
          frames[libfreenect2::Frame::Color]->data).copyTo(bgr_);

      // Undistort the image.
      registration_->apply(
          frames[libfreenect2::Frame::Color],
          frames[libfreenect2::Frame::Depth],
          &undistorted,
          &registered);

      // Construct the undistorted depth image.
      cv::Mat(
          undistorted.height,
          undistorted.width,
          kDepthFormat,
          undistorted.data).copyTo(depth_);

      // Construct the BGR undistorted image.
      cv::Mat(
          registered.height,
          registered.width,
          kColorFormat,
          registered.data).copyTo(bgrUndistorted_);
    }

    listener_.release(frames);

    // Increment the frame counter.
    {
      std::unique_lock<std::mutex> locker(countLock_);
      frameCount_++;
      countLock_.unlock();
      countCond_.notify_all();
    }

  }
}

void KinectCamera::freshFrame() {
  std::unique_lock<std::mutex> locker(countLock_);
  auto oldCount = frameCount_;
  countCond_.wait(locker, [this, oldCount]() {
    return frameCount_ > oldCount;
  });
}

cv::Mat KinectCamera::undistort(
    const cv::Mat &HDImage,
    const cv::Mat &depthImage,
    int format)
{
  // Prepare HD colour and depth frames.
  libfreenect2::Frame HDFrame(kColorImageWidth, kColorImageHeight, 4);
  libfreenect2::Frame depthFrame(kDepthImageWidth, kDepthImageHeight, 4);
  HDFrame.data = HDImage.data;
  depthFrame.data = depthImage.data;

  libfreenect2::Frame depthUndistortedFrame(
      kDepthImageWidth,
      kDepthImageHeight,
      4);
  libfreenect2::Frame HDundistortedFrame(
      kDepthImageWidth,
      kDepthImageHeight,
      4);

  // Undistort images.
  registration_->apply(
      &HDFrame,
      &depthFrame,
      &depthUndistortedFrame,
      &HDundistortedFrame);

  // Construct the BGR graycode undistorted image.
  cv::Mat undistorted;
  cv::Mat(
      HDundistortedFrame.height,
      HDundistortedFrame.width,
      format,
      HDundistortedFrame.data).copyTo(undistorted);

  return undistorted;
}

