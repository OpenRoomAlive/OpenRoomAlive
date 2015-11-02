// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <fstream>

#include <libfreenect2/logger.h>


/**
 * This class is responsible for logging kinect output to a file.
 */
class KinectFileLogger: public libfreenect2::Logger {
 public:
  /**
  * enum Level
  * {
  *   None = 0,
  *   Error = 1,
  *   Warning = 2,
  *   Info = 3,
  *   Debug = 4,
  * };
  **/
  KinectFileLogger(Level logLevel, const std::string &logFilename);
  ~KinectFileLogger();

  void log(Level level, const std::string &message) override;

 private:
  std::ofstream logFile_;
};