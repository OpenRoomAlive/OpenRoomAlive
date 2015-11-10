// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include "core/Master.h"
#include "core/Types.h"

namespace dv { namespace master {

/**
 * Class representing an event associated with laser detection by a ProCam.
 */
class Event {
 public:

  /**
   * Default event with id_ set to unused value - used when stopping execution.
   */
  Event();

  /**
   * Normal, properly specified event.
   */
  Event(const ConnectionID id, const Point point, const int64_t color);

  ~Event();

 private:
  /// ID of the ProCam that sent the event.
  const ConnectionID id_;
  /// Point representing new detected position of the laser.
  const Point point_;
  /// Color of the detected laser.
  const int64_t color_;
};

} }