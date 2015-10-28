// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include "master/ProCam.h"
#include "master/ProCamSystem.h"

using namespace dv::master;

ProCamSystem::ProCamSystem() {
}

ProCamSystem::~ProCamSystem() {
}

std::shared_ptr<ProCam> ProCamSystem::addCamera() {
  auto id = ++nextID_;
  std::shared_ptr<ProCam> proCam;

  {
    // TODO: fix this
    std::unique_lock<std::mutex> locker(lock_);
    auto result = proCams_.emplace(id, std::make_shared<ProCam>(id, 0, 0));
    if (!result.second || result.first == proCams_.end()) {
      throw std::runtime_error("Cannot create procam unit.");
    }

    proCam = result.first->second;
  }

  return proCam;
}
