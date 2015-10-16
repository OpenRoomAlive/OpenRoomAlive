// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <iostream>
#include <thread>
#include <iostream>

#include <boost/make_shared.hpp>

#include <libfreenect2/libfreenect2.hpp>

#include <thrift/protocol/TBinaryProtocol.h>
#include <thrift/transport/TSocket.h>
#include <thrift/transport/TTransportUtils.h>

#include "core/Master.h"
#include "core/Exception.h"
#include "slave/GLDisplay.h"
#include "slave/ProCamApplication.h"


using namespace dv::slave;


ProCamApplication::ProCamApplication(
    const std::string &masterIP,
    uint16_t masterPort)
  : masterIP_(masterIP),
    masterPort_(masterPort),
    display_(new GLDisplay()),
    runProcam_(true)
{
  (void) masterIP_;
  (void) masterPort_;
  (void) runProcam_;
}

int ProCamApplication::run() {
  // Interfacing with master node
  //std::thread networking([this]() {
  //  talkToMaster();
  //});

  display_->run();

  // Kinect.
  // TODO(ilijar): set up the kinect.
  return EXIT_SUCCESS;
}

void ProCamApplication::respondToMaster() {
  // Run server responding to master node requests for data.
}

