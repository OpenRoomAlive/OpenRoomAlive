// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <atomic>

#include <folly/Optional.h>

#include "GrayCode.h"
#include "slave/GLDisplay.h"

namespace dv { namespace slave {

/**
 * Encapsulates most of the functionality of the application.
 */
class ProCamApplication {
 public:
  ProCamApplication(
      const std::string &masterIP,
      uint16_t port,
      bool enableProjector);
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
};

}}
