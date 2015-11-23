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
    const cv::Mat &colorCamMat,
    const cv::Mat &irCamMat,
    const cv::Mat &irDist,
    const cv::Size &actualProjRes,
    const cv::Size &effectiveProjRes,
    const std::chrono::milliseconds &latency)
{
  std::unique_lock<std::mutex> locker(lock_);

  auto result = proCams_.emplace(
      id,
      std::make_shared<ProCam>(
          colorCamMat,
          irCamMat,
          irDist,
          actualProjRes,
          effectiveProjRes,
          latency));

  if (!result.second || result.first == proCams_.end()) {
    throw std::runtime_error("Cannot create procam unit.");
  }
}

std::shared_ptr<const ProCam> ProCamSystem::getProCam(ConnectionID id) const {
  auto it = proCams_.find(id);

  if (it != proCams_.end()) {
    return it->second;
  } else {
    throw EXCEPTION() << "ProCam with a specified ID was not found.";
  }
}

std::shared_ptr<ProCam> ProCamSystem::getProCam(ConnectionID id) {
  return std::const_pointer_cast<ProCam>(
      static_cast<const ProCamSystem&>(*this).getProCam(id));
}

