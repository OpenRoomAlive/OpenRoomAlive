// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include "core/Exception.h"
#include "slave/KinectFileLogger.h"

using namespace dv;

KinectFileLogger::KinectFileLogger(
    Level logLevel,
    const std::string &logFilename)
  : logFile_(logFilename) 
{
  if (!logFile_.is_open()) {
    throw EXCEPTION() << "Opening Kinect log file failed.";
  }
  level_ = logLevel;
}

KinectFileLogger::~KinectFileLogger() {
}

void KinectFileLogger::log(Level level, const std::string &message) {
  if (level <= level_) {
    logFile_ 
      << "[" << libfreenect2::Logger::level2str(level) << "] " << message
      << std::endl;
  }
}