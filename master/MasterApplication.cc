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

#include "master/MasterApplication.h"
#include "master/MasterServerFactory.h"

using namespace dv;
using namespace dv::master;


MasterApplication::MasterApplication(uint16_t port, size_t procamTotal)
  : port_(port),
    procamTotal_(procamTotal)
{
  assert (procamTotal_ > 0);
  (void) port_;
  (void) procamTotal_;
  (void) runMaster_;
}

int MasterApplication::run() {
  // TODO: Add a map? to the class to map incoming connections to outgoing
  //       and pass it as arg to the constructor of MasterServerFactory
  //       Use it to handle connections -> modify getHandler().
  //       You can also pass procamTotal_ to MasterServerFactory constructor to control 
  //       (and reject) ProCams connecting to master

  // TODO2: Use runMaster_ for communication between threads

  // Handle networking with Procams
  std::thread networking([this]() {
   listenToProcams();
  });
  
  // Processing?
  
  // Wait for networking to finish excution
  networking.join();

  return EXIT_SUCCESS;
}

void MasterApplication::listenToProcams() {
  namespace atp = apache::thrift::protocol;
  namespace att = apache::thrift::transport;
  namespace ats = apache::thrift::server;

  ats::TThreadedServer server(
    boost::make_shared<MasterProcessorFactory>(
        boost::make_shared<MasterServerFactory>()),
    boost::make_shared<att::TServerSocket>(port_),
    boost::make_shared<att::TBufferedTransportFactory>(),
    boost::make_shared<atp::TBinaryProtocolFactory>());

  std::cout << "Starting the server..." << std::endl;
  server.serve();
  std::cout << "Done." << std::endl;
}

