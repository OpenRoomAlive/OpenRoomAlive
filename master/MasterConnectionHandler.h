// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <thrift/transport/TTransportUtils.h>

#include "core/Master.h"
#include "core/ProCam.h"
#include "master/MasterServer.h"

namespace dv { namespace master {


/**
 * Manages ProCam connections.
 */
class MasterConnectionHandler : public MasterIfFactory {
 private:
  using TBufferedTransport = apache::thrift::transport::TBufferedTransport;

 public:
  MasterConnectionHandler();
  ~MasterConnectionHandler();

  /**
   * Retrieves the handler and sets up the inverse connection.
   */
  virtual MasterServer* getHandler(
      const ::apache::thrift::TConnectionInfo& connInfo);
  /**
   * Free the handler.
   */
  virtual void releaseHandler(MasterIf* handler);

 private:
  /// Connections to ProCam units.
  std::vector<boost::shared_ptr<TBufferedTransport>> connections_;
  /// ProCam clients.
  std::vector<std::shared_ptr<ProCamClient>> clients_;
};

}}
