// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <iostream>
#include <thread>

#include <boost/make_shared.hpp>

#include <libfreenect2/libfreenect2.hpp>

#include <thrift/protocol/TBinaryProtocol.h>
#include <thrift/server/TThreadedServer.h>
#include <thrift/transport/TSocket.h>
#include <thrift/transport/TTransportUtils.h>

#include "core/Master.h"
#include "core/Exception.h"
#include "slave/GLDisplay.h"
#include "slave/GrayCode.h"
#include "slave/ProCamApplication.h"
#include "slave/ProCamServer.h"

#if defined(KINECT_CAMERA)
  #include "slave/KinectCamera.h"
  using RGBDCameraImpl = dv::slave::KinectCamera;
#elif defined(MOCK_CAMERA)
  #include "slave/MockCamera.h"
  using RGBDCameraImpl = dv::slave::MockCamera;
#else
  #error "No camera implementation provided."
#endif

using namespace dv::slave;


ProCamApplication::ProCamApplication(
    const std::string &masterIP,
    uint16_t port,
    bool enableProjector)
  : masterIP_(masterIP)
  , port_(port)
  , runProcam_(true)
  , grayCode_(new GrayCode())
{
  if (enableProjector) {
    display_.emplace();
  }

  (void) runProcam_;
}

ProCamApplication::~ProCamApplication() {
}

int ProCamApplication::run() {
  // Responding to master node requests
  std::thread networking([this]() {
    serveMaster();
  });

  // Send Procam's IP to master
  namespace at  = apache::thrift;
  namespace atp = apache::thrift::protocol;
  namespace att = apache::thrift::transport;

  auto socket    = boost::make_shared<att::TSocket>(masterIP_, port_);
  auto transport = boost::make_shared<att::TBufferedTransport>(socket);
  auto protocol  = boost::make_shared<atp::TBinaryProtocol>(transport);
  MasterClient masterClient(protocol);

  try {
    transport->open();

    std::cout << "Sending Procam's IP to master node..." << std::endl;
    if (!masterClient.ping()) {
      std::cout << "Master node rejected IP of the Procam." << std::endl;
      return EXIT_FAILURE;
    }

    std::cout << "Master node accepted IP of the Procam." << std::endl;

    transport->close();
  } catch (apache::thrift::TException& tx) {
    std::cout << "ERROR: " << tx.what() << std::endl;
    throw tx;
  }

  // Procam server.
  //slave::ProCamServer proCamServer(std::make_shared<RGBDCameraImpl>());

  if (display_.hasValue()) {
    // TODO(nandor): This is just a test.
    auto image = grayCode_->getPattern(GrayCode::Orientation::HORIZONTAL, 0);

    display_->displayImage(image);
    display_->run();
  } else {
    getchar();
  }

  return EXIT_SUCCESS;
}

void ProCamApplication::serveMaster() {
  namespace atp = apache::thrift::protocol;
  namespace att = apache::thrift::transport;
  namespace ats = apache::thrift::server;

  ats::TThreadedServer server(
      boost::make_shared<ProCamProcessor>(
          // TODO(ilijar): remove null when T1 is done.
          boost::make_shared<ProCamServer>(nullptr)),
      boost::make_shared<att::TServerSocket>(port_ + 1),
      boost::make_shared<att::TBufferedTransportFactory>(),
      boost::make_shared<atp::TBinaryProtocolFactory>());

  std::cout << "Starting the ProCam server..." << std::endl;
  server.serve();
  std::cout << "ProCam server done." << std::endl;
}
