// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <iostream>

#include <boost/shared_ptr.hpp>

#include <thrift/transport/TSocket.h>

#include "master/MasterServer.h"
#include "master/MasterServerFactory.h"

using namespace dv;
using namespace dv::master;


MasterServer* MasterServerFactory::getHandler(
    const ::apache::thrift::TConnectionInfo& connInfo)
{
  namespace att = apache::thrift::transport;

  auto sock = boost::dynamic_pointer_cast<att::TSocket>(connInfo.transport);
  std::cout << "Incoming connection" << std::endl;
  std::cout << "\tSocketInfo: "  << sock->getSocketInfo() << std::endl;
  std::cout << "\tPeerHost: "    << sock->getPeerHost() << std::endl;
  std::cout << "\tPeerAddress: " << sock->getPeerAddress() << std::endl;
  std::cout << "\tPeerPort: "    << sock->getPeerPort() << std::endl;
  return new MasterServer;
}

void MasterServerFactory::releaseHandler(MasterIf* handler) {
  if (handler != nullptr) {
    delete handler;
  }
}
