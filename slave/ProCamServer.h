// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <cstdlib>
#include <string>

#include <libfreenect2/libfreenect2.hpp>

#include "core/ProCam.h"


namespace dv { namespace slave {

class ProCamServer : virtual public dv::ProCamIf {
 public:
  ProCamServer(const std::shared_ptr<libfreenect2::Freenect2Device>& kinect);
  ~ProCamServer();

  /**
   * Retrieves the camera parameters.
   */
  void getCameraParams(CameraParams& cameraParams);

  /**
   * Test.
   */
  int32_t derpderp();

 private:
  /// Kinect device.
  const std::shared_ptr<libfreenect2::Freenect2Device> kinect_;
};

}}
