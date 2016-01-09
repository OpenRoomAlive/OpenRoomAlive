// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <algorithm>

#include "core/Exception.h"
#include "core/Types.h"
#include "master/ProCamSystem.h"

using namespace dv;
using namespace dv::master;


namespace {

template<typename T, size_t D>
cv::Mat loadMat(
    int height,
    int width,
    const folly::dynamic &arr)
{
  if (!arr.isArray()) {
    throw EXCEPTION() << "Array expected in JSON.";
  }

  if (width * height * D != arr.size()) {
    throw EXCEPTION() << "Invalid array size.";
  }
  cv::Mat mat = cv::Mat::zeros({ width, height }, CVType<T, D>::type);
  for (int i = 0, idx = 0; i < height; ++i) {
    auto ptr = mat.ptr<T>(i);
    for (int j = 0; j < width; ++j) {
      for (size_t k = 0; k < D; ++k, ++idx) {
        ptr[j * D + k] = arr[idx].asDouble();
      }
    }
  }

  return mat;
}

folly::dynamic saveMat(const cv::Mat &mat) {
  folly::dynamic arr = {};

  switch (mat.type()) {
    case CV_32FC1: {
      for (int i = 0; i < mat.rows; ++i) {
        auto ptr = mat.ptr<float>(i);
        for (int j = 0; j < mat.cols; ++j) {
          arr.push_back(static_cast<double>(ptr[j]));
        }
      }
      break;
    }
    case CV_64FC1: {
      for (int i = 0; i < mat.rows; ++i) {
        auto ptr = mat.ptr<double>(i);
        for (int j = 0; j < mat.cols; ++j) {
          arr.push_back(ptr[j]);
        }
      }
      break;
    }
    case CV_8UC4: case CV_8UC3: {
      auto depth = mat.channels();
      for (int i = 0; i < mat.rows; ++i) {
        auto ptr = mat.ptr<uint8_t>(i);
        for (int j = 0; j < mat.cols; ++j) {
          for (int k = 0; k < depth; ++k) {
            arr.push_back(ptr[j * depth + k]);
          }
        }
      }
      break;
    }
    default: {
      throw EXCEPTION() << "Unsupported format.";
    }
  }

  return arr;
}

}


ProCamSystem::ProCamSystem() {
}

ProCamSystem::~ProCamSystem() {
}

void ProCamSystem::fromJSON(const folly::dynamic &data) {
  for (const auto &key : data.keys()) {
    const auto &cd = data[key];
    addProCam(
        std::stoi(key.asString().toStdString()),
        {
          loadMat<float, 1>(3, 3, cd["color-cam"]["proj"]),
          loadMat<float, 1>(0, 0, cd["color-cam"]["dist"]),
        },
        {
          loadMat<float, 1>(3, 3, cd["ir-cam"]["proj"]),
          loadMat<float, 1>(1, 5, cd["ir-cam"]["dist"]),
        },
        cv::Size(
            cd["actual-res"]["width"].asInt(),
            cd["actual-res"]["height"].asInt()
        ),
        cv::Size(
            cd["effective-res"]["width"].asInt(),
            cd["effective-res"]["height"].asInt()
        ),
        std::chrono::milliseconds(cd["latency"].asInt())
    );

    const std::shared_ptr<ProCam> cam = getProCam(key.asInt());

    const auto &projector = cd.getDefault("projector");
    cam->projMat_ = loadMat<double, 1>(3, 3, projector["proj"]);
    cam->projDist_ = loadMat<double, 1>(1, 5, projector["dist"]);

    const auto &baseline = cd.getDefault("baseline");
    cam->depthBaseline_ = loadMat<float, 1>(424, 512, baseline["depth"]);
    cam->depthVariance_ = loadMat<float, 1>(424, 512, baseline["var"]);
    cam->colorBaseline_ = loadMat<uint8_t, 4>(424, 512, baseline["color"]);

    const auto &poses = cd.getDefault("poses");
    for (const auto &pose : poses.keys()) {
      const auto &pd = poses[pose];
      const auto &id = pose.asInt();

      cam->projectorGroup_.push_back(id);
      cam->poses_[id] = {
          loadMat<double, 1>(3, 1, pd["rvec"]),
          loadMat<double, 1>(3, 1, pd["tvec"])
      };
    }
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
        )
        ( "color-cam", folly::dynamic::object
            ( "proj", saveMat(cam->colorCam_.calib) )
            ( "dist", saveMat(cam->colorCam_.dist) )
        )
        ( "ir-cam", folly::dynamic::object
            ( "proj", saveMat(cam->irCam_.calib) )
            ( "dist", saveMat(cam->irCam_.dist) )
        )
        ( "projector", folly::dynamic::object
            ( "proj", saveMat(cam->projMat_) )
            ( "dist", saveMat(cam->projDist_) )
        )
        ( "baseline", folly::dynamic::object
            ( "depth", saveMat(cam->depthBaseline_) )
            ( "var", saveMat(cam->depthVariance_) )
            ( "color", saveMat(cam->colorBaseline_ ) )
        )
        ( "latency", cam->latency_.count() )
        ;

    data[std::to_string(id)]["poses"] = folly::dynamic::object;
    for (const auto &next : cam->projectorGroup_) {
      auto pose = cam->getPose(next);
      data[std::to_string(id)]["poses"][std::to_string(next)] =
          folly::dynamic::object
              ( "rvec", saveMat(pose.rvec) )
              ( "tvec", saveMat(pose.tvec) )
              ;
    }
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
          latency
      )
  );

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

