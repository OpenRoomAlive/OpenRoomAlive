// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <iostream>
#include <memory>
#include <random>
#include <vector>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <gtest/gtest.h>

#include <opencv2/opencv.hpp>

#include "core/ProCam.h"
#include "core/Types.h"
#include "master/Calibrator.h"
#include "master/ProCamSystem.h"
#include "test/mock/MockConnectionHandler.h"
#include "test/mock/MockEnvironment.h"

using namespace dv;
using namespace dv::master;
using namespace dv::test;


class CalibratorTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    // Create the conneciton handler.
    const auto &dir = kTestEnv->getDataFile("test0");
    conn = boost::static_pointer_cast<ConnectionHandler>(
        boost::make_shared<MockConnectionHandler>(dir));
    ids = conn->waitForConnections(1);

    // Create a procam system.
    system = std::make_shared<ProCamSystem>();
    for (const auto &param : conn->getParams()) {
      auto &camera = param.second.camera;
      auto &display = param.second;
      system->addProCam(
          param.first,
          conv::thriftCamMatToCvMat(camera.bgr),
          conv::thriftCamMatToCvMat(camera.ir),
          { display.actualRes.width, display.actualRes.height },
          { display.effectiveRes.width, display.effectiveRes.height },
          std::chrono::milliseconds(display.latency));
    }
  }

  virtual void TearDown() {
  }

 protected:
  boost::shared_ptr<ConnectionHandler> conn;
  std::vector<ConnectionID> ids;
  std::shared_ptr<ProCamSystem> system;
};


TEST_F(CalibratorTest, RunCalibration) {
  /*
  Calibrator calibrator(ids, conn, system);
  calibrator.captureBaselines();
  calibrator.displayGrayCodes();
  calibrator.decodeGrayCodes();
  calibrator.calibrate();
  */
}
