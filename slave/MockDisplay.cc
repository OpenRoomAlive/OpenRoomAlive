// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <thread>

#include <termios.h>
#include <sys/select.h>

#include "slave/MockDisplay.h"

using namespace std::literals;
using namespace dv::slave;
using namespace dv;


MockDisplay::MockDisplay()
  : isRunning_(true)
{
}

MockDisplay::~MockDisplay() {
}

void MockDisplay::run() {
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

  while (isRunning_) {
    // If any key was pressed, break.
    fd_set set;
    FD_ZERO(&set);
    FD_SET(fd, &set);
    if (select(fd + 1, &set, NULL, NULL, &tv) > 0) {
      break;
    }
  }

  tcsetattr(fd, TCSANOW, &oldSettings);
}

DisplayParams MockDisplay::getParameters() {
  DisplayParams params;
  params.frameWidth = 1024;
  params.frameHeight = 1024;
  return params;
}

void MockDisplay::stop() {
  isRunning_ = false;
}

void MockDisplay::displayImage(const cv::Mat &image) {
  (void) image;
}
