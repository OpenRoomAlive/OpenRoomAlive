// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include "slave/RGBDCamera.h"

#include <libfreenect2/libfreenect2.hpp>


namespace dv { namespace slave {


/**
 * Camera source interfacing with the kinect using libfreenect2.
 */
class KinectCamera : public RGBDCamera {
 private:
  using Freenect2 = libfreenect2::Freenect2;
  using Freenect2Device = libfreenect2::Freenect2Device;

 public:
  KinectCamera();
  ~KinectCamera();

  cv::Mat getRGBFrame() override;
  cv::Mat getDepthFrame() override;
  CameraParams getParameters() override;

 private:
  /// Freenect2 library pointer.
  std::shared_ptr<Freenect2> freenect_;
  /// Reference to the kinect device.
  std::shared_ptr<Freenect2Device> kinect_;
  /// Serial ID of the kinect.
  std::string serial_;
};

}}
