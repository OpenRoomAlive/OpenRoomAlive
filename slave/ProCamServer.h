// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <cstdlib>
#include <memory>
#include <string>

#include "core/ProCam.h"


namespace dv { namespace slave {

class RGBDCamera;

/**
 * Thrift server that provides information to the master node.
 */
class ProCamServer : virtual public dv::ProCamIf {
 public:
  ProCamServer(const std::shared_ptr<RGBDCamera>& camera);
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
  const std::shared_ptr<RGBDCamera> camera_;
};

}}
