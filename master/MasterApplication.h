// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <memory>

#include <thrift/server/TServer.h>


namespace dv { namespace master {

class MasterConnectionHandler;


/**
 * Encapsulates most of the functionality of the application.
 */
class MasterApplication {
 public:
  MasterApplication(uint16_t port, size_t procamTotal);
  ~MasterApplication();

  int run();

 private:
  /// Port number the server is listening on.
  const uint16_t port_;
  /// Number of procams expected to connect.
  const size_t procamTotal_;
  /// Handles ProCam connections.
  const boost::shared_ptr<MasterConnectionHandler> connectionHandler_;
  /// Thrift server.
  const std::shared_ptr<apache::thrift::server::TServer> server_;
};

}}
