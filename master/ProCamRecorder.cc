// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <fstream>

#include <boost/make_shared.hpp>
#include <boost/filesystem.hpp>

#include <thrift/protocol/TJSONProtocol.h>
#include <thrift/transport/TFileTransport.h>

#include "core/Conv.h"
#include "core/Exception.h"
#include "master/ProCamRecorder.h"

using namespace dv::master;

const std::string ProCamRecorder::kProCamDir = "procam";

const std::unordered_map<ProCamRecorder::RecordedData, std::string,
  ProCamRecorder::DataHasher> ProCamRecorder::kDataDirNames =
    { { RecordedData::COLOR_BASELINE, "baseline_color_frames" }
    , { RecordedData::DEPTH,          "depth_frames" }
    , { RecordedData::DEPTH_BASELINE, "baseline_depth_frames" }
    , { RecordedData::DEPTH_VARIANCE, "depth_variance" }
    , { RecordedData::GRAYSCALE,      "fullHD_frames" }
    , { RecordedData::UNDISTORTED,    "undistorted_frames" }
    , { RecordedData::UNDISTORTED_HD, "undistorted_HD_frames" }
    };

ProCamRecorder::ProCamRecorder(const std::string &recordDirectory)
  : recordDirectory_(recordDirectory)
{
}

ProCamRecorder::~ProCamRecorder() {
}

void ProCamRecorder::saveFrame(
    const cv::Mat &frame,
    ConnectionID id,
    ProCamRecorder::RecordedData dataType)
{
  // Name of the subdirectory of procam{id} in which the frame should be saved.
  const std::string &frameDir = kDataDirNames.find(dataType)->second;

  // Destination at which the depth image will be stored.
  boost::filesystem::path dest(recordDirectory_);
  dest /= (kProCamDir + std::to_string(id));
  dest /= frameDir;

  // Save depth images as OpenEXR and colour images as PNG.
  switch (frame.depth()) {
    case CV_32F:
    case CV_32S:
    {
      const auto &name = frameFileName(frameNum_[id][dataType]++, "yml");
      cv::FileStorage fs((dest / name).string(), cv::FileStorage::WRITE);
      fs << "mat" << frame;
      fs.release();
      break;
    }
    default:
    {
      const auto &name = frameFileName(frameNum_[id][dataType]++, "png");
      imwrite((dest / name).string(), frame);
      break;
    }
  }
}

void ProCamRecorder::saveParam(const ProCamParam &param, ConnectionID id) {
  namespace tt = apache::thrift::transport;
  namespace tp = apache::thrift::protocol;

  // Record camera parameters in top_level/procam{id}/param.json directory.
  boost::filesystem::path dest(recordDirectory_);
  dest /= (kProCamDir + std::to_string(id));
  dest /= "param.json";

  auto transport = boost::make_shared<tt::TFileTransport>(dest.string());
  auto protocol = boost::make_shared<tp::TJSONProtocol>(
      boost::static_pointer_cast<tp::TTransport>(transport));

  param.write(protocol.get());
}

void ProCamRecorder::createRecordDirectories(size_t count) {
  namespace fs = boost::filesystem;

  // Create the top level directory if one does not exist.
  if (!fs::is_directory(recordDirectory_)) {
    boost::filesystem::create_directory(recordDirectory_);
  }

  // For every proCam create a new directory for the recorded data.
  for (size_t id = 0; id < count; id++) {
    //Create a directory for frames recorded for a single procam.
    fs::path recordPath(recordDirectory_);
    recordPath /= (kProCamDir + std::to_string(id));
    fs::create_directory(recordPath);

    // Create a directory for every type of recorded data.
    for (auto enumDirName : kDataDirNames) {
      // boost::filesystem::path uses compiler generated copy constructor.
      fs::path recDataDirPath(recordPath);
      recDataDirPath /= enumDirName.second;
      fs::create_directory(recDataDirPath);
    }
  }

  // Initialise the counters for each type of frame stored in the directories.
  for (size_t id = 0; id < count; id++) {
    frameNum_.push_back(std::unordered_map<RecordedData, size_t, DataHasher>());
    for (auto enumDirName : kDataDirNames) {
      frameNum_[id][enumDirName.first] = 0;
    }
  }
}

std::string ProCamRecorder::frameFileName(
    const size_t number,
    const std::string &extension)
{
  return "frame" + std::to_string(number) + "." + extension;
}
