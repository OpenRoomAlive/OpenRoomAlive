// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <atomic>
#include <cassert>
#include <fstream>
#include <iostream>
#include <thread>

#include <boost/make_shared.hpp>
#include <boost/filesystem.hpp>

#include <folly/json.h>
#include <folly/dynamic.h>

#include <thrift/protocol/TBinaryProtocol.h>
#include <thrift/server/TThreadedServer.h>
#include <thrift/transport/TServerSocket.h>
#include <thrift/transport/TTransportUtils.h>

#include "core/Async.h"
#include "core/Conv.h"
#include "core/Exception.h"
#include "core/GLViewer.h"
#include "master/Calibrator.h"
#include "master/EventStream.h"
#include "master/LaserDrawer.h"
#include "master/MasterApplication.h"
#include "master/MasterConnectionHandler.h"
#include "master/PointCloud.h"
#include "master/ProCamSystem.h"
#include "master/RecordingConnectionHandler.h"

using namespace dv::core;
using namespace dv::master;

constexpr auto kCalibrateFileName = ".calibrate";

MasterApplication::MasterApplication(
    uint16_t port,
    size_t procamTotal,
    const std::string &recordDirectory,
    bool calibrate,
    bool render,
    bool twoStepK)
  : stream_(new EventStream())
  , port_(port)
  , procamTotal_(procamTotal)
  , connectionHandler_(recordDirectory.empty()
        ? new MasterConnectionHandler(port_ + 1, stream_)
        : new RecordingConnectionHandler(port_ + 1, stream_, recordDirectory))
  , server_(new apache::thrift::server::TThreadedServer(
        boost::make_shared<MasterProcessorFactory>(connectionHandler_),
        boost::make_shared<apache::thrift::transport::TServerSocket>(port_),
        boost::make_shared<apache::thrift::transport::TBufferedTransportFactory>(),
        boost::make_shared<apache::thrift::protocol::TBinaryProtocolFactory>()))
  , system_(new ProCamSystem())
  , calibrate_(calibrate)
  , render_(render)
  , twoStepK_(twoStepK)
{
  if (procamTotal_ == 0) {
    throw EXCEPTION() << "At least one procam should be attached.";
  }
}

MasterApplication::~MasterApplication() {
}

int MasterApplication::run() {
  // Spawn a thread that runs the server.
  std::cout << "Starting server." << std::endl;
  auto futureServer = asyncExecute([this] () {
    server_->serve();
  });

  // Wait for all the procams to connect.
  std::cout << "Waiting for " << procamTotal_ << " connections..." << std::endl;
  auto connectionIds = connectionHandler_->waitForConnections(procamTotal_);
  std::cout << "All " << procamTotal_ << " ProCams connected." << std::endl;

  // Check if there is a valid calibration file or if user explicitly
  // asked to re-calibrate the system from scratch.
  auto calibFile = boost::filesystem::current_path() / kCalibrateFileName;
  if (calibrate_ || !boost::filesystem::is_regular(calibFile)) {
    std::cout << "Calibrating ProCam System..." << std::endl;

    // Create an empty ProCam system.
    for (const auto &param : connectionHandler_->getParams()) {
      auto camera = param.second.camera;
      auto display = param.second;

      system_->addProCam(
          param.first,
          conv::thriftCamMatToCvMat(camera.bgr),
          conv::thriftCamMatToCvMat(camera.ir),
          { display.actualRes.width, display.actualRes.height },
          { display.effectiveRes.width, display.effectiveRes.height },
          std::chrono::milliseconds(display.latency));
    }

    // Calibrate the system using grey codes.
    Calibrator calibrator(connectionIds, connectionHandler_, system_);
    calibrator.captureBaselines();
    calibrator.formProjectorGroups();
    calibrator.displayGrayCodes();
    calibrator.decodeGrayCodes();
    calibrator.calibrate(twoStepK_);

    std::cerr << "Calibration completed." << std::endl;

    // Write the calibration data to .calibrate.
    std::ofstream(calibFile.string())
        << folly::toJson(system_->toJSON())
        << std::endl;
  } else {
    std::cout << "Loading calibration data..." << std::endl;

    // Read the calibration data from the .calibrate file.
    std::ifstream file(calibFile.string());
    std::string source(
        (std::istreambuf_iterator<char>(file)),
        std::istreambuf_iterator<char>());
    system_->fromJSON(folly::parseJson(source));
  }

  // Perform 3D reconstruction.
  PointCloud pointCloud(connectionIds, system_);
  pointCloud.construct();

  std::future<void> futureInput;

  if (render_) {
    GLViewer viewer([&pointCloud, &viewer] () {
      viewer.drawPoints(pointCloud.getPoints(), pointCloud.getCentroid());
    });
  } else {
    // Spawn thread waiting for user input - input => quit.
    futureInput = asyncExecute([&, this] () {
      getchar();
      stream_->close();
    });

    // Perform laser drawing.
    connectionHandler_->clearDisplays();
    connectionHandler_->startLaserDetection();
    LaserDrawer(connectionIds, system_, stream_, connectionHandler_).run();
  }

  // Disconnect all clients.
  connectionHandler_->stop();

  // Wait for networking to finish excution.
  server_->stop();
  futureServer.get();
  futureInput.get();

  return EXIT_SUCCESS;
}

