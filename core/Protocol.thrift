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

/**
 * Definition of the API exposed by the ProCam unit.
 */
service ProCam {
  /**
   * Retrieves the parameters of the Color and Ir cameras.
   */
  CameraParams getCameraParams(),

  // TODO(ilijar): T1

  // Test.
  i32 derpderp()
}

/**
 * Definition of the API exposed by the Server.
 */
service Master {

  // Send Procam's IP to master node.
  bool ping()
}
