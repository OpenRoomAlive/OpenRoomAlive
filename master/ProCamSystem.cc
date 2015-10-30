// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include "master/ProCamSystem.h"


using namespace dv::master;

ProCamSystem::ProCamSystem() {
}

ProCamSystem::~ProCamSystem() {
}

void ProCamSystem::addProCam(
    ConnectionID id,
    const dv::CameraParams &camParams,
    const dv::DisplayParams &displayParams)
{
  std::unique_lock<std::mutex> locker(lock_);

  auto result = proCams_.emplace(id,
      std::make_shared<ProCam>(camParams, displayParams));

  if (!result.second || result.first == proCams_.end()) {
    throw std::runtime_error("Cannot create procam unit.");
  }
}

std::shared_ptr<const ProCam> ProCamSystem::getProCam(ConnectionID id) {
  std::unique_lock<std::mutex> locker(lock_);

  auto it = proCams_.find(id);

  if (it != proCams_.end()) {
    return it->second;
  } else {
    throw EXCEPTION() << "ProCam with a specified ID was not found.";
  }
}
