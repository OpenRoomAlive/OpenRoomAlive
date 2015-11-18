// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <atomic>

#include "procam/Display.h"


namespace dv { namespace procam {


/**
 * Class encapsulating all the OpenGL rendering logic.
 */
class MockDisplay : public Display {
 public:
  MockDisplay();
  ~MockDisplay();

  bool isRunning() override;
  void update() override;
  void stop() override;
  void displayImage(const cv::Mat &image) override;
  cv::Size getParameters() override;
  virtual size_t getWidth() override;
  virtual size_t getHeight() override;

 private:
  std::atomic<bool> isRunning_;
};

}}
