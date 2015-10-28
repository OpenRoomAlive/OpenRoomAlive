// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <cassert>
#include <iostream>
#include <thread>

#include <boost/make_shared.hpp>

#include <thrift/protocol/TBinaryProtocol.h>
#include <thrift/server/TThreadedServer.h>
#include <thrift/transport/TServerSocket.h>
#include <thrift/transport/TTransportUtils.h>

#include "core/Async.h"
#include "core/Exception.h"
#include "master/MasterApplication.h"
#include "master/MasterConnectionHandler.h"
#include "master/Calibrator.h"

using namespace dv::master;


MasterApplication::MasterApplication(uint16_t port, size_t procamTotal)
  : port_(port)
  , procamTotal_(procamTotal)
  , connectionHandler_(new MasterConnectionHandler(port_ + 1))
  , server_(new apache::thrift::server::TThreadedServer(
        boost::make_shared<MasterProcessorFactory>(connectionHandler_),
        boost::make_shared<apache::thrift::transport::TServerSocket>(port_),
        boost::make_shared<apache::thrift::transport::TBufferedTransportFactory>(),
        boost::make_shared<apache::thrift::protocol::TBinaryProtocolFactory>()))
{
  if (procamTotal_ <= 0) {
    throw EXCEPTION() << "At least one procam should be attached.";
  }
}

MasterApplication::~MasterApplication() {
}

int MasterApplication::run() {
  // Spawn a thread that runs the server.
  std::cout << "Starting server." << std::endl;
  auto future = asyncExecute([this] () {
    server_->serve();
  });

  // Wait for all the procams to connect.
  std::cout << "Waiting for " << procamTotal_ << " connections." << std::endl;
  auto connectionIds = connectionHandler_->waitForConnections(procamTotal_);

  std::cout << "IDs of the connected procams:" << std::endl;
  for (const auto &id : connectionIds) {
    std::cout << "  " << id << std::endl;
  }

  Calibrator calibrator(connectionIds, connectionHandler_);

  // Display the gray code patterns for calibration.
  calibrator.displayGrayCodes();

  // Wait for user input.
  getchar();

  // Disconnect all clients.
  connectionHandler_->stop();

  // Wait for networking to finish excution.
  server_->stop();
  future.get();

  return EXIT_SUCCESS;
}
