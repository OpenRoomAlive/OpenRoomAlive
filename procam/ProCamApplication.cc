// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <iostream>
#include <string>
#include <thread>

#include <boost/filesystem.hpp>
#include <boost/make_shared.hpp>

#include <libfreenect2/libfreenect2.hpp>

#include <thrift/protocol/TBinaryProtocol.h>
#include <thrift/server/TSimpleServer.h>
#include <thrift/transport/TSocket.h>

#include "core/Async.h"
#include "core/Conv.h"
#include "core/Exception.h"
#include "core/GrayCode.h"
#include "core/Master.h"
#include "procam/BaselineCapture.h"
#include "procam/Display.h"
#include "procam/GLDisplay.h"
#include "procam/KinectCamera.h"
#include "procam/LaserDetector.h"
#include "procam/MockCamera.h"
#include "procam/MockDisplay.h"
#include "procam/ProCamApplication.h"

using namespace dv;
using namespace dv::procam;

using namespace std::literals;
using namespace apache::thrift;


// Max delay before giving up on a connection.
constexpr auto kMaxConnectWait = 120s;

// Sleep time between checks if it is safe to break connections.
constexpr auto kWaitForStreamShutdown = 100ms;

static inline GrayCode::Orientation orientationCast(
    Orientation::type orientation)
{
  switch (orientation) {
    case Orientation::type::HORIZONTAL: {
      return GrayCode::Orientation::HORIZONTAL;
    }
    case Orientation::type::VERTICAL: {
      return GrayCode::Orientation::VERTICAL;
    }
    default: {
      throw EXCEPTION() << "Failed to cast enum of thrift Orientation::type "
                           "to GrayCode::Orientation.";
    }
  }
}

ProCamApplication::ProCamApplication(
    const std::string &masterIP,
    uint16_t port,
    bool enableDisplay,
    bool enableKinect,
    bool enableMaster,
    uint16_t logLevel,
    const std::string &logFilename,
    const Size &effectiveSize,
    uint32_t latency)
  : masterIP_(masterIP)
  , port_(port)
  , display_(enableDisplay
        ? static_cast<Display*>(new GLDisplay())
        : static_cast<Display*>(new MockDisplay()))
  , camera_(enableKinect
        ? static_cast<BGRDCamera*>(new KinectCamera(logLevel, logFilename))
        : static_cast<BGRDCamera*>(new MockCamera(logLevel, logFilename)))
  , grayCode_(effectiveSize.width, effectiveSize.height)
  , server_(new server::TSimpleServer(
        boost::make_shared<ProCamProcessor>(
            boost::shared_ptr<ProCamApplication>(this, [](auto*){})),
        boost::make_shared<transport::TServerSocket>(port_ + 1),
        boost::make_shared<transport::TBufferedTransportFactory>(),
        boost::make_shared<protocol::TBinaryProtocolFactory>()))
  , transport_(new transport::TBufferedTransport(
        boost::make_shared<transport::TSocket>(masterIP_, port_)))
  , master_(new MasterClient(boost::make_shared<protocol::TBinaryProtocol>(transport_)))
  , baseline_(new BaselineCapture())
  , enableMaster_(enableMaster)
  , latency_(latency)
  , updatesStreamOn_(true)
  , detectingLaser_(false)
{
}

ProCamApplication::~ProCamApplication() {
}

int ProCamApplication::run() {
  camera_->freshFrame();

  if (enableMaster_) {
    // Responding to master node requests.
    auto futureServer = asyncExecute([this]() {
      server_->serve();
    });

    // Open the connection with master and ping it.
    pingMaster();

    // Using the open connection, start sending updates of laser positions.
    auto futureLaser = asyncExecute([this]() {
      detectLaser();
    });

    // Continuously update display and depth baseline.
    while (display_->isRunning()) {
      baseline_->process(camera_->getDepthImage());
      display_->update();
    }

    // Wait with breaking connections until it is safe.
    while (updatesStreamOn_) {
      std::this_thread::sleep_for(kWaitForStreamShutdown);
    }

    // Stop everything.
    transport_->close();
    server_->stop();
    std::cerr << "Disconnected from master." << std::endl;
    futureServer.get();
    futureLaser.get();
  } else {
    // Debug stuff here.
    cv::namedWindow("test");
    while (cv::waitKey(1) != 'q') {
      auto depth = camera_->getDepthImage();
      baseline_->process(depth);
      cv::imshow("test", baseline_->getDepthImage() / 5000.0f);
    }
  }

  return EXIT_SUCCESS;
}

void ProCamApplication::getParam(ProCamParam& params) {
  params.camera = camera_->getParameters();

  auto actual = display_->getResolution();
  params.actualRes.width = actual.width;
  params.actualRes.height = actual.height;

  auto effective = grayCode_.getResolution();
  params.effectiveRes.width = effective.width;
  params.effectiveRes.height = effective.height;

  params.latency = latency_;
}

void ProCamApplication::getColorImage(Frame& frame) {
  camera_->freshFrame();
  cv::Mat image = camera_->getColorImage();
  conv::cvMatToThriftFrame(image, frame);
}

void ProCamApplication::getDepthImage(Frame& frame) {
  camera_->freshFrame();
  cv::Mat image = camera_->getDepthImage();
  conv::cvMatToThriftFrame(image, frame);
}

void ProCamApplication::getUndistortedColorImage(Frame& frame) {
  camera_->freshFrame();
  cv::Mat image = camera_->getUndistortedColorImage();
  conv::cvMatToThriftFrame(image, frame);
}

void ProCamApplication::getColorBaseline(Frame &frame) {
  camera_->freshFrame();
  // TODO(ilijar): figure out how to account for lighting.
  cv::Mat image = camera_->getUndistortedColorImage();
  conv::cvMatToThriftFrame(image, frame);
}

void ProCamApplication::getDepthBaseline(Frame &frame) {
  baseline_->framesProcessed();
  conv::cvMatToThriftFrame(baseline_->getDepthImage(), frame);
}

void ProCamApplication::getDepthVariance(Frame &frame) {
  baseline_->framesProcessed();
  conv::cvMatToThriftFrame(baseline_->getDepthVariance(), frame);
}

void ProCamApplication::displayGrayCode(
    const Orientation::type orientation,
    const int16_t level,
    bool invertedGrayCode)
{
  cv::Mat grayCodeImage = grayCode_.getPattern(
      orientationCast(orientation),
      static_cast<size_t>(level));

  std::cout << "Displaying gray code pattern: " << level << std::endl;

  // Display the inverted gray code if the flag is set.
  if (invertedGrayCode) {
    cv::Mat inverted;
    cv::bitwise_not(grayCodeImage, inverted);
    display_->displayImage(inverted);
  } else {
    display_->displayImage(grayCodeImage);
  }
}

void ProCamApplication::displayWhite() {
  display_->displayImage(cv::Mat(1, 1, CV_8UC3, cv::Scalar(160, 160, 160)));
}

void ProCamApplication::clearDisplay() {
  display_->displayImage(cv::Mat::zeros(display_->getResolution(), CV_8UC3));
}

void ProCamApplication::close() {
  display_->stop();
}

void ProCamApplication::undistort(
    Frame& undistortedImageThrift,
    const Frame& HDImageThrift)
{
  // Get HD as cv::Mat
  cv::Mat HDimage;
  conv::thriftFrameToCvMat(HDImageThrift, HDimage);
  // Ask for undistortion and send back to master
  conv::cvMatToThriftFrame(
      camera_->undistort(HDimage, baseline_->getDepthImage()),
      undistortedImageThrift);
}

void ProCamApplication::updateLaser(
    const std::vector<Segment> &segments,
    const Color &color)
{
  display_->updateWithLaser(
      conv::thriftSegmentsToCvPoints(segments),
      conv::thriftColorToCvScalar(color));
}

void ProCamApplication::pingMaster() {
  // Open a connection. If it fails, wait using binary exponential backoff.
  for (auto wait = 1s, total = 0s; total < kMaxConnectWait; total += wait) {
    try {
      transport_->open();
      break;
    } catch (apache::thrift::TException& tx) {
      std::cerr <<
          "Connection failed. Retrying in " << wait.count() << "s" << std::endl;
      std::this_thread::sleep_for(wait);
      if (wait < 5s) {
        wait += 1s;
      }
    }
  }
  if (!transport_->isOpen()) {
    throw EXCEPTION() << "Cannot connect to master.";
  }

  // Ping the master.
  if (!master_->ping()) {
    throw EXCEPTION() << "Error on master.";
  }
  std::cout << "Connected to master." << std::endl;
}

void ProCamApplication::startLaserDetection() {
  std::unique_lock<std::mutex> locker(detectionLock_);
  detectingLaser_ = true;
  detectionLock_.unlock();
  detectionCond_.notify_all();
}

void ProCamApplication::detectLaser() {
  // Start detecting the laser after receiving the signal from the master.
  {
    std::unique_lock<std::mutex> locker(detectionLock_);
    detectionCond_.wait(locker, [this] () {
      return detectingLaser_;
    });
  }

  LaserDetector detector(display_, master_, camera_, baseline_);
  detector.detect();

  updatesStreamOn_ = false;
}
