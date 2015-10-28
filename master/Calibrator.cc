// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <algorithm>
#include <thread>
#include <chrono>
#include <iostream>

#include "core/ProCam.h"
#include "core/GrayCode.h"
#include "master/Calibrator.h"

using namespace dv::master;
using namespace dv;

using namespace std::chrono_literals;

// Duration of a one element of gray code sequence in milliseconds.
constexpr auto kGrayCodeDuration = 1000ms;

Calibrator::Calibrator(
    const std::vector<ConnectionID>& ids,
    const boost::shared_ptr<MasterConnectionHandler>& connectionHandler,
    const std::shared_ptr<ProCamSystem>& system)
  : ids_(ids)
  , connectionHandler_(connectionHandler)
  , system_(system)
{
}

Calibrator::~Calibrator() {
}

void Calibrator::displayGrayCodes() {
  for (const auto &id : ids_) {
    auto displayParams = system_->getDisplayParams(id);

    std::cout << "Display params for ProCam with ID: " << id << ". Params: "
              << displayParams.frameWidth << " " << displayParams.frameHeight
              << std::endl;

    size_t level = std::min(
        slave::GrayCode::calculateLevel(displayParams.frameWidth),
        slave::GrayCode::calculateLevel(displayParams.frameHeight));

    std::cout << "  Levels: " << level << std::endl;

    for (size_t i = 0; i < level; i++) {
      // Display interchangebly the vertical and horizontal patterns.
      connectionHandler_->displayGrayCode(id, Orientation::type::VERTICAL, i);

      std::this_thread::sleep_for(kGrayCodeDuration);

      connectionHandler_->displayGrayCode(id, Orientation::type::HORIZONTAL, i);

      std::this_thread::sleep_for(kGrayCodeDuration);
    }
  }
}
