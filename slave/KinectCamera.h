// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <atomic>
#include <mutex>

#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/registration.h>

#include "slave/RGBDCamera.h"


namespace dv { namespace slave {

namespace {

  const size_t kColorImageWidth = 1920;
  const size_t kColorImageHeight = 1080;
  const size_t kDepthImageWidth = 512;
  const size_t kDepthImageHeight = 424;
  const size_t kBytesPerPixel = 4;

} // anonymous namespace


/**
 * Camera source interfacing with the kinect using libfreenect2.
 */
class KinectCamera : public RGBDCamera {
 private:
  using Freenect2 = libfreenect2::Freenect2;
  using PacketPipeline = libfreenect2::PacketPipeline;
  using Freenect2Device = libfreenect2::Freenect2Device;
  using SyncMultiFrameListener = libfreenect2::SyncMultiFrameListener;
  using Registration = libfreenect2::Registration;
  using FrameMap = libfreenect2::FrameMap;
  using Frame = libfreenect2::Frame;

 public:
  KinectCamera();
  ~KinectCamera();

  /**
   * Retrieves undistorted depth image.
   */
  cv::Mat getDepthFrame() override;

  /**
   * Retrieves RGB color image.
   */
  cv::Mat getRGBFrame() override;

  /**
   * Retrieves camera parameters.
   */
  CameraParams getParameters() override;

 private:
  /**
   * Retrieves frames from the device.
   */
  void poll();

 private:
  /// Freenect2 library pointer.
  std::shared_ptr<Freenect2> freenect_;
  /// Reference to the kinect pipeline.
  std::shared_ptr<PacketPipeline> pipeline_;
  /// Reference to the kinect device.
  std::shared_ptr<Freenect2Device> kinect_;
  /// Kinect listener.
  SyncMultiFrameListener listener_;
  /// Kinect registration.
  std::shared_ptr<Registration> registration_;
  /// RGB color image (1920x1080).
  cv::Mat rgb_;
  /// Undistorted depth image.
  Frame undistorted_;
  /// Color image for the depth data (512x424).
  Frame registered_;
  /// Mutex guarding the frames.
  std::mutex framesLock_;
  /// Serial ID of the kinect.
  std::string serial_;
  /// Thread for polling the Kinect data.
  std::thread dataPolling_;
  /// Flag indicating if the kinect is running.
  std::atomic<bool> isRunning_;
};

}}

