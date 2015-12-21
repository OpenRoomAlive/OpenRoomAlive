// This file is part of the Group 13 Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

namespace cpp dv


struct Resolution {
  1: i16 width;
  2: i16 height;
}


/**
 * BGR camera parameters.
 */
struct BGRCameraParams {
   1: double fx,
   2: double fy,
   3: double cx,
   4: double cy,

   5: double shift_d,
   6: double shift_m,

   7: double mx_x3y0,
   8: double mx_x0y3,
   9: double mx_x2y1,
  10: double mx_x1y2,
  11: double mx_x2y0,
  12: double mx_x0y2,
  13: double mx_x1y1,
  14: double mx_x1y0,
  15: double mx_x0y1,
  16: double mx_x0y0,

  17: double my_x3y0,
  18: double my_x0y3,
  19: double my_x2y1,
  20: double my_x1y2,
  21: double my_x2y0,
  22: double my_x0y2,
  23: double my_x1y1,
  24: double my_x1y0,
  25: double my_x0y1,
  26: double my_x0y0,
}


/**
 * Depth camera parameters.
 */
struct IrCameraParams {
  1: double fx,
  2: double fy,
  3: double cx,
  4: double cy,

  5: double k1,
  6: double k2,
  7: double k3,
  8: double p1,
  9: double p2,
}


/**
 * Parameters of the Kinect2 camera.
 */
struct CameraParams {
  1: BGRCameraParams bgr,
  2: IrCameraParams ir,
}


/**
 * Parameters of all the elements of a procam.
 */
struct ProCamParam {
  // Parameters of the projector.
  1: Resolution actualRes,
  2: Resolution effectiveRes,
  3: i32 latency,

  // Camera parameters.
  4: CameraParams camera,
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
 * Depth Point for laser tracking.
 */
struct Point {
  1: double x;
  2: double y;
  3: double depth;
}

/**
 * Segment of laser path in projected image.
 */
struct Segment {
  1: i16 x1;
  2: i16 y1;
  3: i16 x2;
  4: i16 y2;
}

/**
 * Color in RGB.
 */
struct Color {
  1: i16 r;
  2: i16 g;
  3: i16 b;
}

/**
 * Definition of the API exposed by the ProCam unit.
 */
service ProCam {
  /**
   * Retrieves the parameters of the Color and IR cameras and the display.
   */
  ProCamParam getParam(),

  /**
   * Retrieves the grayscale image (1920x1080).
   */
  Frame getGrayscaleImage(),

  /**
   * Retrieves the undistorted depth image (512x424).
   */
  Frame getDepthImage(),

  /**
   * Retrieves the BGR image for depth data (512x424).
   */
  Frame getUndistortedColorImage(),

  /**
   * Retireves the color baseline.
   */
  Frame getColorBaseline();

  /**
   * Retrieves the depth baseline.
   */
  Frame getDepthBaseline(),

  /**
   * Retrieves the depth variance.
   */
  Frame getDepthVariance(),

  /**
   * Displays the specified gray code pattern.
   */
  void displayGrayCode(
      1: Orientation orientation,
      2: i16 level,
      3: bool invertedGrayCode),

  /**
   * Display a white image on projector.
   */
  void displayWhite();

  /**
   * Clears the display (sets it to a black image).
   */
  void clearDisplay();

  /**
   * Closes the procam app.
   */
  oneway void close();

  /**
   * Undistorts provided HD image using Kinect's parameters and depth baseline.
   */
  Frame undistort(1: Frame inputHDImageThrift),

  /**
   * Starts detecting the laser.
   */
  void startLaserDetection();

  /**
   * Update in image the laser path of color 'color' with provided 'segments'.
   */
  void updateLaser(1: list<Segment> segments, 2: Color color);
}

/**
 * Definition of the API exposed by the Server.
 */
service Master {
  /**
   * Send Procam's IP to master node.
   */
  bool ping()

  /**
   * Sends a new laser position to master.
   */
  void detectedLaser(1: Point point, 2: Color color);
}
