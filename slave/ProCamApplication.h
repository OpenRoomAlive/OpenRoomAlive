// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <atomic>

#include <folly/Optional.h>

#include <thrift/server/TServer.h>

#include "slave/GrayCode.h"
#include "slave/GLDisplay.h"


namespace dv { namespace slave {

class RGBDCamera;


/**
 * Encapsulates most of the functionality of the application.
 */
class ProCamApplication {
 public:
  ProCamApplication(
      const std::string &masterIP,
      uint16_t port,
      bool enableProjector,
      bool enableKinect);
  ~ProCamApplication();

  int run();

 private:
  /**
   * Handles master requests.
   */
  void serveMaster();

 private:
  /// IP of the master node.
  const std::string masterIP_;
  /// Port used for conenctions.
  const uint16_t port_;
  /// OpenGL window.
  folly::Optional<GLDisplay> display_;
  /// Flag for threads to message each other when Procam is to be shut down.
  std::atomic<bool> runProcam_;
  /// Gray code generator
  const std::shared_ptr<GrayCode> grayCode_;
  /// Server instance.
  const std::shared_ptr<apache::thrift::server::TServer> server_;
  /// Kinect camera implementation.
  const std::shared_ptr<RGBDCamera> camera_;
};

}}
