// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/registration.h>

#include "slave/RGBDCamera.h"


namespace dv { namespace slave {

namespace {

  const size_t kFrameWidth = 512;
  const size_t kFrameHeight = 424;
  const size_t kFramePixelSize = 4;

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

  cv::Mat getRGBFrame() override;
  cv::Mat getDepthFrame() override;
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
  /// Frames.
  FrameMap frames_;
  /// Undistorted frame.
  Frame undistorted_;
  /// Registered frame.
  Frame registered_;
  /// Serial ID of the kinect.
  std::string serial_;
};

}}
