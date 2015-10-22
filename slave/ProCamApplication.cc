// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <iostream>
#include <thread>

#include <boost/make_shared.hpp>

#include <libfreenect2/libfreenect2.hpp>

#include <thrift/protocol/TBinaryProtocol.h>
#include <thrift/server/TSimpleServer.h>
#include <thrift/transport/TSocket.h>
#include <thrift/transport/TTransportUtils.h>

#include "core/Master.h"
#include "core/Exception.h"
#include "slave/Display.h"
#include "slave/GLDisplay.h"
#include "slave/GrayCode.h"
#include "slave/MockCamera.h"
#include "slave/MockDisplay.h"
#include "slave/ProCamApplication.h"

#if defined(KINECT_CAMERA)
  #include "slave/KinectCamera.h"
  using RGBDCameraImpl = dv::slave::KinectCamera;
#else
  using RGBDCameraImpl = dv::slave::MockCamera;
#endif

using namespace dv::slave;
using namespace std::literals;


/**
 * Max delay before giving up on a connection.
 */
constexpr auto MAX_CONNECT_WAIT = 512s;


ProCamApplication::ProCamApplication(
    const std::string &masterIP,
    uint16_t port,
    bool enableDisplay,
    bool enableKinect)
  : masterIP_(masterIP)
  , port_(port)
  , grayCode_(new GrayCode())
  , server_(new apache::thrift::server::TSimpleServer(
        boost::make_shared<ProCamProcessor>(
            boost::shared_ptr<ProCamApplication>(this, [](ProCamApplication*){})),
        boost::make_shared<apache::thrift::transport::TServerSocket>(port_ + 1),
        boost::make_shared<apache::thrift::transport::TBufferedTransportFactory>(),
        boost::make_shared<apache::thrift::protocol::TBinaryProtocolFactory>()))
  , display_(enableDisplay
        ? static_cast<Display*>(new GLDisplay())
        : static_cast<Display*>(new MockDisplay()))
  , camera_(enableKinect
        ? static_cast<RGBDCamera*>(new RGBDCameraImpl())
        : static_cast<RGBDCamera*>(new MockCamera()))
{
}

ProCamApplication::~ProCamApplication() {
}

int ProCamApplication::run() {
  // Responding to master node requests
  std::thread networking([this]() {
    server_->serve();
  });

  // Send Procam's IP to master
  namespace at  = apache::thrift;
  namespace atp = apache::thrift::protocol;
  namespace att = apache::thrift::transport;

  auto socket    = boost::make_shared<att::TSocket>(masterIP_, port_);
  auto transport = boost::make_shared<att::TBufferedTransport>(socket);
  auto protocol  = boost::make_shared<atp::TBinaryProtocol>(transport);
  MasterClient masterClient(protocol);

  // Open a connection. If it fails, wait using binary exponential backoff.
  for (auto wait = 1s; wait < MAX_CONNECT_WAIT; wait += wait) {
    try {
      transport->open();
      break;
    } catch (apache::thrift::TException& tx) {
      std::cerr << "Connection failed. Retrying in " << wait.count() << "s." << std::endl;
      std::this_thread::sleep_for(wait);
    }
  }
  if (!transport->isOpen()) {
    throw EXCEPTION() << "Cannot connect to master.";
  }

  // Ping the master.
  if (!masterClient.ping()) {
    return EXIT_FAILURE;
  }
  std::cout << "Connected to master." << std::endl;
  transport->close();

  // Run the display.
  display_->run();

  // Stop everything.
  std::cerr << "Disconnected from master." << std::endl;
  networking.join();
  return EXIT_SUCCESS;
}

void ProCamApplication::getCameraParams(CameraParams& cameraParams) {
  cameraParams = camera_->getParameters();
}

void ProCamApplication::close() {
  server_->stop();
  display_->stop();
}

