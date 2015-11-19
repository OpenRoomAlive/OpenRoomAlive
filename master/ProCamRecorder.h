// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <string>

#include <opencv2/opencv.hpp>

class ProCamRecorder {
 public:
  explicit ProCamRecorder(const std::string &recordDirectory);
  ~ProCamRecorder();

  /// If --record flag was set, color image will be recorded.
  void trySaveColorFrame(const cv::Mat &frame);

  /// If --record flag was set, depth image will be recorded.
  void trySaveDepthFrame(const cv::Mat &frame);

  /// If --record flag was set, undistorted image will be recorded.
  void trySaveUndistortedFrame(const cv::Mat &frame);

  /// If --record flag was set, color baseline image will be recorded.
  void trySaveColorBaselineFrame(const cv::Mat &frame);

  /// If --record flag was set, depth baseline image will be recorded.
  void trySaveDepthBaselineFrame(const cv::Mat &frame);

 private:
  /**
   * Saves, given that the --record option was specified, the given frame in the
   * relevant frameDirectorty, a subdirectory of the recordDirectory_ and names
   * it as frame{frameNumber}.bmp. If --record option was not specified, nothing
   * happens.
   */
  void trySaveFrame(
      const std::string &frameDir,
      size_t frameNumber,
      const cv::Mat &frame);

  /**
   * If the recordDirectory_ field is not empty, the function checks if the
   * specified directory exists, if not, one is created. In that directory,
   * subdirectories for color images, depth images, undistorted images,
   * baseline depth images and baseline color images are created, and all
   * frames captured during proCam operation are persisted.
   */
  void createRecordDirectory();

  /**
   * Given the frame number and extension, the function returns the name of the
   * frame file.
   */
  std::string frameFileName(size_t number, const std::string &extension);

 private:
  /// Directory in which captured frames for all procams are recorded.
  const std::string recordDirectory_;
  /// Number of color frames recorded.
  size_t colorRecorded_;
  /// Number of depth frames recorded.
  size_t depthRecorded_;
  /// Number of undistorted frames recorded.
  size_t undistortedRecorded_;
  /// Number of baseline color frames recorded.
  size_t baselineColorRecorded_;
  /// Number of baseline depth frames recorded.
  size_t baselineDepthRecorded_;
};
