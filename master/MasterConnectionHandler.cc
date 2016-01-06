// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <iostream>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <thrift/protocol/TBinaryProtocol.h>
#include <thrift/transport/TSocket.h>
#include <thrift/transport/TTransportUtils.h>

#include "core/Exception.h"
#include "master/MasterServer.h"
#include "master/MasterConnectionHandler.h"

using namespace dv::master;
using namespace dv;

MasterConnectionHandler::MasterConnectionHandler(
    uint16_t proCamPort,
    const std::shared_ptr<EventStream>& stream)
  : nextID_(0)
  , proCamPort_(proCamPort)
  , stream_(stream)
  , isRunning_(true)
{
}

MasterConnectionHandler::~MasterConnectionHandler() {
  // Close the ProCam connections.
  for (const auto& connection : connections_) {
    try {
      connection.second.client->close();
      connection.second.transport->close();
    } catch (...) {
    }
  }
}

MasterServer*
MasterConnectionHandler::getHandler(const TConnectionInfo& connInfo) {
  namespace at  = apache::thrift;
  namespace atp = apache::thrift::protocol;
  namespace att = apache::thrift::transport;

  auto sock = boost::dynamic_pointer_cast<att::TSocket>(connInfo.transport);

  // Set up a reverse connection.
  auto socket = boost::make_shared<att::TSocket>(
      sock->getPeerAddress(), proCamPort_);
  auto transport = boost::make_shared<att::TBufferedTransport>(socket);
  auto protocol  = boost::make_shared<atp::TBinaryProtocol>(transport);

  ConnectionID id;

  // Open the reverse connection.
  try {
    transport->open();

    // Add the connection.
    {
      std::lock_guard<std::mutex> locker(lock_);
      id = nextID_++;
      connections_.emplace(id, Connection(transport,
          std::make_shared<ProCamClient>(protocol)));
    }

    std::cout << "ProCam "<< id << " connected." << std::endl;
    connectionCountCondition_.notify_all();
  } catch (at::TException& tx) {
    std::cout << "OPENING PROCAM CONNECTION: " << tx.what() << std::endl;
    throw;
  }

  return new MasterServer(stream_, id);
}

void MasterConnectionHandler::releaseHandler(MasterIf* handler) {
  if (handler != nullptr) {
    delete handler;
  }
  if (isRunning_) {
    throw EXCEPTION() << "Connection closed prematurely.";
  }
}

std::vector<ConnectionID>
MasterConnectionHandler::waitForConnections(size_t count) {
  std::unique_lock<std::mutex> locker(lock_);
  auto self = shared_from_this();

  connectionCountCondition_.wait(locker, [count, self, this] () {
    return connections_.size() == count;
  });

  std::vector<ConnectionID> ids;
  for (const auto &connection : connections_) {
    ids.push_back(connection.first);
  }
  return ids;
}

void MasterConnectionHandler::stop() {
  isRunning_ = false;
  std::lock_guard<std::mutex> locker(lock_);
  for (const auto &connection : connections_) {
    try {
      connection.second.client->close();
      connection.second.transport->close();
    } catch (const std::exception &ex) {
      std::cerr << "[Exception] " << ex.what() << std::endl;
    }
  }
  connections_.clear();
}

std::vector<ConnectionID> MasterConnectionHandler::getAllIDs() {
  std::vector<ConnectionID> ids;
  {
    std::lock_guard<std::mutex> locker(lock_);
    for (const auto &connection : connections_) {
      ids.emplace_back(connection.first);
    }
  }
  return ids;
}

std::vector<std::pair<ConnectionID, MasterConnectionHandler::Connection>>
MasterConnectionHandler::getConnections(const std::vector<ConnectionID> &ids) {
  std::vector<std::pair<ConnectionID, Connection>> connections;
  {
    std::lock_guard<std::mutex> locker(lock_);
    for (const auto &id : ids) {
      auto it = connections_.find(id);
      if (it == connections_.end()) {
        throw EXCEPTION() << "Invalid connection ID.";
      }
      connections.emplace_back(*it);
    }
  }
  return connections;
}
