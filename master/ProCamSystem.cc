// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include "master/ProCamSystem.h"


using namespace dv::master;

ProCamSystem::ProCamSystem() {
}

ProCamSystem::~ProCamSystem() {
}

void ProCamSystem::fromJSON(const folly::dynamic &data) {
  for (const auto &key : data.keys()) {
    const auto &cam = data[key];
    addProCam(
        std::stoi(key.asString().toStdString()),
        { cv::Mat(3, 3, CV_32F), cv::Mat(0, 0, CV_32F) },
        { cv::Mat(3, 3, CV_32F), cv::Mat(1, 5, CV_32F) },
        cv::Size(
            cam["actual-res"]["width"].asInt(),
            cam["actual-res"]["height"].asInt()),
        cv::Size(
            cam["effective-res"]["width"].asInt(),
            cam["effective-res"]["height"].asInt()),
        std::chrono::milliseconds(0));
  }
}

folly::dynamic ProCamSystem::toJSON() const {
  folly::dynamic data = folly::dynamic::object;
  for (const auto &kv : proCams_) {
    const auto id = kv.first;
    const auto cam = kv.second;

    data[std::to_string(id)] = folly::dynamic::object
        ( "actual-res", folly::dynamic::object
            ( "width", cam->actualProjRes_.width )
            ( "height", cam->actualProjRes_.height )
        )
        ( "effective-res", folly::dynamic::object
            ( "width", cam->effectiveProjRes_.width )
            ( "height", cam->effectiveProjRes_.height )
        );
  }

  return data;
}

void ProCamSystem::addProCam(
    ConnectionID id,
    const CameraModel &colorCam,
    const CameraModel &irCam,
    const cv::Size &actualProjRes,
    const cv::Size &effectiveProjRes,
    const std::chrono::milliseconds &latency)
{
  std::unique_lock<std::mutex> locker(lock_);

  auto result = proCams_.emplace(
      id,
      std::make_shared<ProCam>(
          colorCam,
          irCam,
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

