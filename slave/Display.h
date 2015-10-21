// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <opencv2/opencv.hpp>


namespace dv { namespace slave {


/**
 * Class encapsulating all the OpenGL rendering logic.
 */
class Display {
 public:
  virtual ~Display();

  /**
   * Main display loop.
   */
  virtual void run() = 0;

  /**
   * Sets the image displayed on the screen.
   * @param image [description]
   */
  virtual void displayImage(const cv::Mat &image) = 0;
};

}}
