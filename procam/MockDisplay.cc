// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <thread>

#include <termios.h>
#include <sys/select.h>

#include "procam/MockDisplay.h"

using namespace std::literals;
using namespace dv::procam;
using namespace dv;


MockDisplay::MockDisplay()
  : isRunning_(true)
{
}

MockDisplay::~MockDisplay() {
}

bool MockDisplay::isRunning() {
  return isRunning_;
}

void MockDisplay::update() {
  struct termios oldSettings;
  struct termios newSettings;
  const int fd = fileno(stdin);

  // Timeout value for polling.
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 500000;

  // Allow stdin to be polled.
  tcgetattr(fd, &oldSettings);
  newSettings = oldSettings;
  newSettings.c_lflag &= ~ICANON & ~ECHO;
  tcsetattr(fd, TCSANOW, &newSettings);

  // If any key was pressed, break.
  fd_set set;
  FD_ZERO(&set);
  FD_SET(fd, &set);
  if (select(fd + 1, &set, NULL, NULL, &tv) > 0) {
    isRunning_ = false;
  }

  // Reset attributes.
  tcsetattr(fd, TCSANOW, &oldSettings);
}

cv::Size MockDisplay::getResolution() {
  cv::Size resolution;
  resolution.width = getWidth();
  resolution.height = getHeight();
  return resolution;
}

size_t MockDisplay::getWidth() {
  return 1024;
}

size_t MockDisplay::getHeight() {
  return 1024;
}

void MockDisplay::stop() {
  isRunning_ = false;
}

void MockDisplay::displayImage(const cv::Mat &image) {
  (void) image;
}

void MockDisplay::updateWithLaser(
    const std::vector<std::pair<cv::Point2i, cv::Point2i>> &segments,
    const cv::Scalar &color) {
  (void) segments;
  (void) color;
}
