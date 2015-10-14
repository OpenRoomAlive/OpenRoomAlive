// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include "stdint.h"

#include "core/ProCam.h"


namespace dv { namespace slave {

class ProCamServer : virtual public dv::ProCamIf {
 public:
  ProCamServer();
  ~ProCamServer();

  /**
   * Test.
   */
  int32_t derpderp();
};

}}
