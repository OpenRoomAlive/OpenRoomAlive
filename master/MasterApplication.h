// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <memory>

#include <thrift/server/TServer.h>


namespace dv { namespace master {

class EventStream;
class MasterConnectionHandler;
class ProCamSystem;

/**
 * Encapsulates most of the functionality of the application.
 */
class MasterApplication {
 public:
  MasterApplication(
      uint16_t port,
      size_t procamTotal,
      const std::string &recordDirectory,
      bool calibrate,
      bool render,
      bool twoStepK);
  ~MasterApplication();

  int run();

 private:
  /// Stream of events sent by ProCams for processing.
  const std::shared_ptr<EventStream> stream_;
  /// Port number the server is listening on.
  const uint16_t port_;
  /// Number of procams expected to connect.
  const size_t procamTotal_;
  /// Handles ProCam connections.
  const boost::shared_ptr<MasterConnectionHandler> connectionHandler_;
  /// Thrift server.
  const std::shared_ptr<apache::thrift::server::TServer> server_;
  /// ProCam system.
  const std::shared_ptr<ProCamSystem> system_;
  /// Indicates if the system need to be re-calibrated.
  const bool calibrate_;
  /// Indicates if the reconstructed 3D mesh is to be rendered.
  const bool render_;
  /// Determines how to compute calibration matrices.
  const bool twoStepK_;
};

}}
