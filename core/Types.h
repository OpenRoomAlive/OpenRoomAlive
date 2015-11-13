// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <cmath>


namespace dv {

/**
 * All connections to procams are identified by ID equal to the ID of the
 * associated ProCam.
 */
using ConnectionID = uint64_t;

/**
 * Screen resolution.
 */
struct Resolution {
  size_t width;
  size_t height;
};

/**
 * Structure representing the equation of a plane.
 */
struct Plane {
  float nx, ny, nz;
  float d;
};


static inline bool equals(float x, float y) {
  const float epsilon = 0.000001f;
  return std::abs(x - y) <= epsilon * std::abs(x);
}

}

