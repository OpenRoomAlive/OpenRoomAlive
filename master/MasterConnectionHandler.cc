// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <iostream>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <thrift/protocol/TBinaryProtocol.h>
#include <thrift/transport/TSocket.h>
#include <thrift/transport/TTransportUtils.h>

#include "master/MasterServer.h"
#include "master/MasterConnectionHandler.h"

using namespace dv::master;

MasterConnectionHandler::MasterConnectionHandler() {
}

MasterConnectionHandler::~MasterConnectionHandler() {
  // Close the ProCam connections.
  for (const auto& connection : connections_) {
    try {
     connection->close();
    } catch (apache::thrift::TException& tx) {
      std::cout << "CLOSING PROCAM CONNECTION: " << tx.what() << std::endl;
    }
  }
}


MasterServer* MasterConnectionHandler::getHandler(
    const ::apache::thrift::TConnectionInfo& connInfo)
{
  namespace at  = apache::thrift;
  namespace atp = apache::thrift::protocol;
  namespace att = apache::thrift::transport;

  auto sock = boost::dynamic_pointer_cast<att::TSocket>(connInfo.transport);
  std::cout << "Incoming connection" << std::endl;

  // Set up a reverse connection.
  auto socket = boost::make_shared<att::TSocket>(
      // TODO(ilijar): hardcoded + 1
      sock->getPeerAddress(), 11631); //sock->getPeerPort());
  auto transport = boost::make_shared<att::TBufferedTransport>(socket);
  auto protocol  = boost::make_shared<atp::TBinaryProtocol>(transport);

  // Open the reverse connection.
  try {
    transport->open();

    // Add the connection.
    connections_.emplace_back(transport);
    clients_.emplace_back(new ProCamClient(protocol));

    // Test
    clients_.back()->derpderp();

  } catch (at::TException& tx) {
    std::cout << "OPENING PROCAM CONNECTION: " << tx.what() << std::endl;
    throw;
  }

  return new MasterServer();
}

void MasterConnectionHandler::releaseHandler(MasterIf* handler) {
  if (handler != nullptr) {
    delete handler;
  }
}
