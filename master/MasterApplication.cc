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
#include <folly/Dynamic.h>

#include <thrift/protocol/TBinaryProtocol.h>
#include <thrift/server/TThreadedServer.h>
#include <thrift/transport/TServerSocket.h>
#include <thrift/transport/TTransportUtils.h>

#include "core/Async.h"
#include "core/Conv.h"
#include "core/Exception.h"
#include "master/Calibrator.h"
#include "master/GLViewer.h"
#include "master/MasterApplication.h"
#include "master/MasterConnectionHandler.h"
#include "master/RecordingConnectionHandler.h"

using namespace dv::master;

constexpr auto kCalibrateFileName = ".calibrate";

MasterApplication::MasterApplication(
    uint16_t port,
    size_t procamTotal,
    const std::string &recordDirectory,
    bool calibrate)
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
  std::cout << "Procams completed." << std::endl;

  // Check if there is a valid calibration file or if user explicitly
  // asked to re-calibrate the system from scratch.
  auto calibFile = boost::filesystem::current_path() / kCalibrateFileName;
  if (calibrate_ || !boost::filesystem::is_regular(calibFile)) {
    std::cout << "Calibrating ProCam System..." << std::endl;

    // Create an empty ProCam system.
    auto camerasParams  = connectionHandler_->getCamerasParams();
    auto displaysParams = connectionHandler_->getDisplaysParams();
    for (const auto &id : connectionIds) {
      auto cameraParam = camerasParams[id];
      auto displayParam = displaysParams[id];
      system_->addProCam(
          id,
          conv::thriftCamMatToCvMat(cameraParam.colorCamMat),
          conv::thriftCamMatToCvMat(cameraParam.irCamMat),
          conv::thriftDistToCvMat(cameraParam.irDist),
          conv::thriftResolutionToCvSize(displayParam.actualRes),
          conv::thriftResolutionToCvSize(displayParam.effectiveRes),
          std::chrono::milliseconds(displayParam.latency));
    }

    // Calibrate the system using grey codes.
    Calibrator calibrator(connectionIds, connectionHandler_, system_);
    calibrator.captureBaselines();
    calibrator.displayGrayCodes();
    calibrator.decodeGrayCodes();
    calibrator.calibrate();

    std::cerr << "Calibration completed." << std::endl;

    // Write the calibration data to .calibrate.
    std::ofstream(calibFile.string())
        << folly::toPrettyJson(system_->toJSON()).toStdString()
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

  // Spawn thread waiting for user input - input => quit.
  std::atomic<bool> run = {true};
  auto futureInput = asyncExecute([&, this] () {
    getchar();
    run = false;
    // Send to itself a breaking ("empty") event.
    stream_->push(Event());
  });

  // Process events from Procams.
  while (run) {
    auto event = stream_->poll();
    (void) event;
    // TODO: Process event using the 3D reconstruction.
    // TODO: Send updates to Procams (1 or many per event)
  }

  // Disconnect all clients.
  connectionHandler_->stop();

  // Wait for networking to finish excution.
  server_->stop();
  futureServer.get();
  futureInput.get();

  return EXIT_SUCCESS;
}

