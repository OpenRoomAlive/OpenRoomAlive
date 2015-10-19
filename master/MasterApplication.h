// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once


namespace dv { namespace master {

/**
 * Encapsulates most of the functionality of the application.
 */
class MasterApplication {
 public:
  MasterApplication(uint16_t port, size_t procamTotal);

  int run();

 private:
  // Handle networking with Procams
  void listenToProcams();

 private:
  /// Port number the server is listening on.
  const uint16_t port_;
  /// Number of procams expected to connect.
  const size_t procamTotal_;
  /// Flag for threads to message each other when master node is to be shut down.
  std::atomic<bool> runMaster_;
};

}}
