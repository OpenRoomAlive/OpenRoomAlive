// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include "slave/Display.h"


namespace dv { namespace slave {


/**
 * Class encapsulating all the OpenGL rendering logic.
 */
class MockDisplay : public Display {
 public:
  ~MockDisplay();

  /**
   * Main display loop.
   */
  void run() override;

  /**
   * Sets the image displayed on the screen.
   * @param image [description]
   */
  void displayImage(const cv::Mat &image) override;
};

}}
