// This file is part of the Group 13 Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

namespace cpp dv


/**
 * f - focal point
 * c - principal point
 */
struct ColorParams {
  1: double fx;
  2: double fy;
  3: double cx;
  4: double cy;
}

/**
 * f - focal point
 * c - principal point
 * k - tangential disortion
 * p - radial disortion
 */
struct IrParams {
  1: double fx;
  2: double fy;
  3: double cx;
  4: double cy;
  5: double k1;
  6: double k2;
  7: double k3;
  8: double p1;
  9: double p2;
}

struct CameraParams {
  1: ColorParams color;
  2: IrParams ir;
}

struct DisplayParams {
  1: i16 frameWidth;
  2: i16 frameHeight;
}

struct Frame {
  1: i32 rows;
  2: i32 cols;
  3: i64 format;
  4: string data;
}

enum Orientation {
  HORIZONTAL = 0,
  VERTICAL   = 1
}

/**
 * Definition of the API exposed by the ProCam unit.
 */
service ProCam {
  /**
   * Retrieves the parameters of the Color and Ir cameras.
   */
  CameraParams getCameraParams(),

  /**
   * Retrieves the parameters of the display.
   */
  DisplayParams getDisplayParams(),

  /**
   * Retrieves the BGR image (1920x1080).
   */
  Frame getColorImage(),

  /**
   * Retrieves the undistorted depth image (512x424).
   */
  Frame getDepthImage(),

  /**
   * Retrieves the BGR image for depth data (512x424).
   */
  Frame getUndistortedColorImage(),

  /**
   * Displays the specified gray code pattern.
   */
  void displayGrayCode(1: Orientation orientation, 2: i16 level),

  /**
   * Closes the procam app.
   */
  oneway void close()
}

/**
 * Definition of the API exposed by the Server.
 */
service Master {
  /**
   * Send Procam's IP to master node.
   */
  bool ping()
}
