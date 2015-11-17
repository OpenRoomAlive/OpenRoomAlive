// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <boost/filesystem.hpp>

#include "procam/BaselineCapture.h"
#include "master/ProCamRecorder.h"

constexpr auto kColorDir     = "color_frames";
constexpr auto kDepthDir     = "depth_frames";
constexpr auto kUndisDir     = "undistorted_frames";
constexpr auto kBaseColorDir = "baseline_color_frames";
constexpr auto kBaseDepthDir = "baseline_depth_frames";

ProCamRecorder::ProCamRecorder(const std::string &recordDirectory)
    : recordDirectory_(recordDirectory)
    , colorRecorded_(0)
    , depthRecorded_(0)
    , undistortedRecorded_(0)
    , baselineColorRecorded_(0)
    , baselineDepthRecorded_(0)
{
  createRecordDirectory();
}

void ProCamRecorder::trySaveColorFrame(const cv::Mat &frame)  {
  trySaveFrame(kColorDir, colorRecorded_++, frame);
}

void ProCamRecorder::trySaveDepthFrame(const cv::Mat &frame) {
  trySaveFrame(kDepthDir, depthRecorded_++, frame);
}

void ProCamRecorder::trySaveUndistortedFrame(const cv::Mat &frame) {
  trySaveFrame(kUndisDir, undistortedRecorded_++, frame);
}

void ProCamRecorder::trySaveColorBaselineFrame(const cv::Mat &frame) {
  trySaveFrame(kColorDir, baselineColorRecorded_++, frame);
}

void ProCamRecorder::trySaveDepthBaselineFrame(const cv::Mat &frame) {
  trySaveFrame(kBaseDepthDir, baselineDepthRecorded_++, frame);
}

void ProCamRecorder::trySaveFrame(
    const std::string &frameDir,
    size_t frameNumber,
    const cv::Mat &frame)
{
  if (!recordDirectory_.empty()) {
    // Destination at which the depth image will be stored.
    boost::filesystem::path dest(recordDirectory_);
    dest /= frameDir;
    dest /= frameFileName(frameNumber, "bmp");

    imwrite(dest.string(), frame);
  }
}

void ProCamRecorder::createRecordDirectory() {
  namespace fs = boost::filesystem;

  // Create a new directory for the recorded data.
  if (!recordDirectory_.empty()) {
    if (!fs::is_directory(recordDirectory_)) {
      boost::filesystem::create_directory(recordDirectory_);
    }
    // Create subdirectories for the three kinds of frames.
    fs::create_directory(
        recordDirectory_ + fs::path::preferred_separator + kColorDir);
    fs::create_directory(
        recordDirectory_ + fs::path::preferred_separator + kDepthDir);
    fs::create_directory(
        recordDirectory_ + fs::path::preferred_separator + kUndisDir);
    fs::create_directory(
        recordDirectory_ + fs::path::preferred_separator + kBaseColorDir);
    fs::create_directory(
        recordDirectory_ + fs::path::preferred_separator + kBaseDepthDir);
  }
}

std::string ProCamRecorder::frameFileName(
    const size_t number,
    const std::string &extension)
{
  return ("frame" + std::to_string(number) + "." + extension);
}