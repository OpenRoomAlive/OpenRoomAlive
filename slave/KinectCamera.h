// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <atomic>
#include <condition_variable>
#include <mutex>

#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/registration.h>

#include "slave/BGRDCamera.h"
#include "slave/KinectFileLogger.h"


namespace dv { namespace slave {


/**
 * Camera source interfacing with the kinect using libfreenect2.
 */
class KinectCamera : public BGRDCamera {
 public:
  KinectCamera(uint16_t logLevel, const std::string &logFile);
  ~KinectCamera();

  /**
   * Retrieves the BGR color image.
   */
  cv::Mat getColorImage() override;

  /**
   * Retrieves the undistorted depth image.
   */
  cv::Mat getDepthImage() override;

  /**
   * Retrieves the color image for depth data.
   */
  cv::Mat getUndistortedColorImage() override;

  /**
   * Retrieves camera parameters.
   */
  CameraParams getParameters() override;

  /**
   * Blocks until a fresh frame is retrieved.
   */
  void freshFrame() override;

 private:
  /**
   * Retrieves frames from the device.
   */
  void poll();

 private:
  /// Freenect2 library pointer.
  libfreenect2::Freenect2 *freenect_;
  /// Reference to the kinect pipeline.
  libfreenect2::PacketPipeline *pipeline_;
  /// Reference to the kinect device.
  libfreenect2::Freenect2Device *kinect_;
  /// Kinect listener.
  libfreenect2::SyncMultiFrameListener listener_;
  /// Kinect registration.
  libfreenect2::Registration *registration_;
  /// Logger of Kinect messages.
  std::shared_ptr<KinectFileLogger> logger_;
  /// BGR color image (1920x1080).
  cv::Mat bgr_;
  /// Undistorted depth image (512x424).
  cv::Mat depth_;
  /// Undistorted BGR color image for depth data (512x424).
  cv::Mat bgrUndistorted_;
  /// Mutex guarding the frames.
  std::mutex framesLock_;
  /// Serial ID of the kinect.
  std::string serial_;
  /// Thread for polling the Kinect data.
  std::thread dataPolling_;
  /// Flag indicating if the kinect is running.
  std::atomic<bool> isRunning_;
  /// Number of frames retrieved.
  uint64_t frameCount_;
  /// Condition variable waiting on a fresh frame.
  std::condition_variable countCond_;
  /// Mutex protecting the frame count.
  std::mutex countLock_;
};

}}

