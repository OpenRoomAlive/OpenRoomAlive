// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <string>
#include <unordered_map>

#include <opencv2/opencv.hpp>

#include "core/Protocol_types.h"
#include "core/Types.h"

namespace dv { namespace master {

class ProCamRecorder
{
 public:
  enum class RecordedData;

 public:
  explicit ProCamRecorder(const std::string &recordDirectory);
  ~ProCamRecorder();

  /**
   * Saves the given frame in the relevant frameDirectory, a subdirectory of
   * the procam{id}, and names it as frame{frameNumber}.bmp.
   */
  void saveFrame(const cv::Mat &frame, ConnectionID id, RecordedData frameType);

  /**
   * Records camera parameters in the directory corresponding to a given proCam.
   */
  void saveCameraParams(const cv::Mat &frame, ConnectionID id);

  /**
   * Records display parameters in the directory corresponding to a given
   * proCam.
   */
  void saveDisplayParams(const DisplayParams &params, ConnectionID id);

  /**
   * If the recordDirectory_ field is not empty, the function checks if the
   * specified directory exists, if not, one is created. In that directory,
   * a subdirectory for each connectde proCam is created. Within each of those
   * subdirectories for color images, depth images, undistorted images,
   * baseline depth images and baseline color images are created, and all
   * frames captured during proCam operation are persisted.
   */
  void createRecordDirectories(size_t count);

 public:
  /**
   * Enum class representing the types of data recorded by the recorder.
   */
  enum class RecordedData {
    COLOR,
    COLOR_BASELINE,
    DEPTH,
    DEPTH_BASELINE,
    DEPTH_VARIANCE,
    UNDISTORTED,
    UNDISTORTED_HD,
    CAMERA_PARAMS,
    DISPLAY_PARAMS
  };

  struct DataHasher {
    std::size_t operator() (const RecordedData data) const {
      return static_cast<std::size_t>(data);
    }
  };

  /**
   * Names of the directories in which the recorded data is stored. Order of data
   * type names should be the same like in the RecordedData enum.
   */
  const std::unordered_map<RecordedData, std::string, DataHasher> kDataDirNames
      { { RecordedData::COLOR,          "color_frames" }
      , { RecordedData::COLOR_BASELINE, "baseline_color_frames" }
      , { RecordedData::DEPTH,          "depth_frames" }
      , { RecordedData::DEPTH_BASELINE, "baseline_depth_frames" }
      , { RecordedData::DEPTH_VARIANCE, "depth_variance" }
      , { RecordedData::UNDISTORTED,    "undistorted_frames" }
      , { RecordedData::UNDISTORTED_HD, "undistorted_HD_frames" }
      , { RecordedData::CAMERA_PARAMS,  "camera_params" }
      , { RecordedData::DISPLAY_PARAMS, "display_params" }
      };

 private:
  /**
   * Given the frame number and extension, the function returns the name of the
   * frame file.
   */
  std::string frameFileName(size_t number, const std::string &extension);

 private:
  /// Directory in which captured frames for all procams are recorded.
  const std::string recordDirectory_;
  /// Index of the next of a certain frame to be saved.
  std::vector<std::unordered_map<RecordedData, size_t, DataHasher>> frameNum_;
};

}}
