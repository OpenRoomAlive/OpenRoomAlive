// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <iostream>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <thrift/protocol/TBinaryProtocol.h>
#include <thrift/transport/TSocket.h>
#include <thrift/transport/TTransportUtils.h>

#include "core/Conv.h"
#include "core/Exception.h"
#include "master/MasterServer.h"
#include "master/MasterConnectionHandler.h"

using namespace dv::master;
using namespace dv;

MasterConnectionHandler::MasterConnectionHandler(uint16_t proCamPort)
  : proCamPort_(proCamPort)
  , nextID_(0)
{
}

MasterConnectionHandler::~MasterConnectionHandler() {
  // Close the ProCam connections.
  for (const auto& connection : connections_) {
    try {
     connection.second.transport->close();
    } catch (apache::thrift::TException& tx) {
      std::cout << "CLOSING PROCAM CONNECTION: " << tx.what() << std::endl;
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

  // Open the reverse connection.
  try {
    transport->open();

    // Add the connection.
    {
      std::lock_guard<std::mutex> locker(lock_);
      auto id = nextID_++;
      connections_.emplace(id, Connection(transport,
          std::make_shared<ProCamClient>(protocol)));
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

void MasterConnectionHandler::displayGrayCode(
    ConnectionID id,
    Orientation::type orientation,
    int16_t level,
    bool invertedGrayCode)
{
  InvokeOne(id, &ProCamClient::displayGrayCode, orientation, level, invertedGrayCode);
}

void MasterConnectionHandler::displayWhite(ConnectionID id) {
  InvokeOne(id, &ProCamClient::displayWhite);
}

void MasterConnectionHandler::clearDisplay(ConnectionID id) {
  InvokeOne(id, &ProCamClient::clearDisplay);
}

void MasterConnectionHandler::stop() {
  std::lock_guard<std::mutex> locker(lock_);
  for (const auto &connection : connections_) {
    connection.second.client->close();
  }
}

std::unordered_map<ConnectionID, cv::Mat>
MasterConnectionHandler::getColorImages() {
  return conv::thriftFrameToCvMatMap(
      InvokeParallel(&ProCamClient::getColorImage));
}

std::unordered_map<ConnectionID, cv::Mat>
MasterConnectionHandler::getDepthImages() {
  return conv::thriftFrameToCvMatMap(
      InvokeParallel(&ProCamClient::getDepthImage));
}

std::unordered_map<ConnectionID, cv::Mat>
MasterConnectionHandler::getUndistortedColorImages() {
  return conv::thriftFrameToCvMatMap(
      InvokeParallel(&ProCamClient::getUndistortedColorImage));
}

std::unordered_map<ConnectionID, cv::Mat>
MasterConnectionHandler::getColorBaselines() {
  return conv::thriftFrameToCvMatMap(
      InvokeParallel(&ProCamClient::getColorBaseline));
}

std::unordered_map<ConnectionID, cv::Mat>
MasterConnectionHandler::getDepthBaselines() {
  return conv::thriftFrameToCvMatMap(
      InvokeParallel(&ProCamClient::getDepthBaseline));
}

