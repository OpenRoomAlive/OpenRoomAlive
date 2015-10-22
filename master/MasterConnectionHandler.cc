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
     connection.transport->close();
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
    {
      std::lock_guard<std::mutex> locker(lock_);
      connections_.emplace_back(transport, std::make_shared<ProCamClient>(protocol));
    }

    std::cout << "ProCam connected." << std::endl;
    connectionCountCondition_.notify_all();
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

void MasterConnectionHandler::waitForConnections(size_t count) {
  std::unique_lock<std::mutex> locker(lock_);
  auto self = shared_from_this();
  connectionCountCondition_.wait(locker, [count, self, this] () {
    return connections_.size() == count;
  });
}

void MasterConnectionHandler::stop() {
  std::lock_guard<std::mutex> locker(lock_);
  for (const auto &connection : connections_) {
    connection.client->close();
  }
}
