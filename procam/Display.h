// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <opencv2/opencv.hpp>

#include "core/Types.h"


namespace dv { namespace procam {


/**
 * Class encapsulating all the OpenGL rendering logic.
 */
class Display {
 public:
  virtual ~Display();

  /**
   * Returns true when the display stops running.
   */
  virtual bool isRunning() = 0;

  /**
   * Main display loop.
   */
  virtual void update() = 0;

  /**
   * Closes the display.
   */
  virtual void stop() = 0;

  /**
   * Retrieves the parameters of the display.
   */
  virtual Resolution getParameters() = 0;

  /**
   * Retrieves the width.
   */
  virtual size_t getWidth() = 0;

  /**
   * Retrieves the height.
   */
  virtual size_t getHeight() = 0;

  /**
   * Sets the image displayed on the screen.
   */
  virtual void displayImage(const cv::Mat &image) = 0;

};

}}
