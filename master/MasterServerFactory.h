// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include "core/Master.h"
#include "master/MasterServer.h"

namespace dv { namespace master {

/**
 * Gives new MasterServer per incoming connection.
 * Allows for decorating the factory and, hence, passing args from
 * main thread to the factory - aids with handling connections.
 */
class MasterServerFactory : virtual public MasterIfFactory {
 public:
  virtual MasterServer* getHandler(
    const ::apache::thrift::TConnectionInfo& connInfo
  );
  virtual void releaseHandler(MasterIf* handler);
};

}}