// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <libfreenect2/libfreenect2.hpp>


namespace dv { namespace slave {

class GLDisplay;

/**
 * Encapsulates most of the functionality of the application.
 */
class ProCamApplication {
 public:
  ProCamApplication(const std::string &masterIP, uint16_t masterPort);

  int run();

 private:
  // Run server responding to master node requests for data.
  void respondToMaster(
      const std::shared_ptr<libfreenect2::Freenect2Device>& kinect);

 private:
  /// IP of the master node.
  const std::string masterIP_;
  /// Port on which Procam messages master node.
  const uint16_t masterPort_;
  /// OpenGL window.
  const std::shared_ptr<GLDisplay> display_;
  /// Flag for threads to message each other when Procam is to be shut down.
  std::atomic<bool> runProcam_;
};

}}
