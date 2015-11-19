// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <fstream>

#include <boost/filesystem.hpp>

#include "master/ProCamRecorder.h"

using namespace dv::master;

constexpr auto kProcamDir = "procam";

ProCamRecorder::ProCamRecorder(const std::string &recordDirectory)
  : recordDirectory_(recordDirectory)
{
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
  dest /= (kProcamDir + std::to_string(id));
  dest /= frameDir;
  dest /= frameFileName(frameNum_[id][dataType]++, "bmp");

  imwrite(dest.string(), frame);
}

void ProCamRecorder::saveCameraParams(const cv::Mat &frame, ConnectionID id)
{
  // Record camera parameters in top_level/procma{id}/camera_params directory.
  boost::filesystem::path dest(recordDirectory_);
  dest /= (kProcamDir + std::to_string(id));

  // Name of the subdirectory in which data about camera parameters are stored.
  dest /= (kDataDirNames.find(RecordedData::CAMERA_PARAMS)->second);

  // Name of the new file that will be created to save parameters of the camera.
  dest /= "cameraParams.xml";

  cv::FileStorage fs(dest.string(), cv::FileStorage::WRITE);
  fs << "CameraParameters" << frame;
}

void ProCamRecorder::saveDisplayParams(
    const DisplayParams &params,
    ConnectionID id)
{
  // Record camera parameters in top_level/procma{id}/displaya_params directory.
  boost::filesystem::path dest(recordDirectory_);
  dest /= (kProcamDir + std::to_string(id));

  // Name of the subdirectory in which data about dsiplay parameters are stored.
  dest /= (kDataDirNames.find(RecordedData::DISPLAY_PARAMS)->second);

  // Name of the new file in which parameters of the display will be saved.
  dest /= "displayParams.txt";

  // Save the display parameters to a text file.
  std::ofstream displayParamsRecord;
  displayParamsRecord.open(dest.string());
  displayParamsRecord << params.effectiveRes.width << std::endl
                      << params.effectiveRes.height;
  displayParamsRecord.close();
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
    recordPath /= (kProcamDir + std::to_string(id));
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
  return ("frame" + std::to_string(number) + "." + extension);
}
