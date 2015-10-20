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
    uint16_t masterPort,
    bool enableProjector)
  : masterIP_(masterIP)
  , masterPort_(masterPort)
  , runProcam_(true)
{
  if (enableProjector) {
    display_.emplace();
  }

  (void) masterIP_;
  (void) masterPort_;
  (void) runProcam_;
}

int ProCamApplication::run() {
  // Responding to master node requests
  //std::thread networking([this]() {
   //respondToMaster();
  //});

  // Send Procam's IP to master
  namespace at  = apache::thrift;
  namespace atp = apache::thrift::protocol;
  namespace att = apache::thrift::transport;

  auto socket    = boost::make_shared<att::TSocket>(masterIP_, masterPort_);
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
  }

  // Procam server.
  //slave::ProCamServer proCamServer(std::make_shared<RGBDCameraImpl>());


  if (display_.hasValue()) {
    // TODO(nandor): This is just a test.
    uint8_t data[] = {255, 0, 0, 128, 128, 128, 128, 128, 128, 128, 255, 0, 0};
    cv::Mat mat(2, 2, CV_8UC3, data);
    display_->displayImage(mat);
    display_->run();
  } else {
    getchar();
  }

  return EXIT_SUCCESS;
}
