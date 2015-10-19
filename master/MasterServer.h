// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include "core/Master.h"

namespace dv { namespace master {

class MasterServer : virtual public MasterIf {
 public:
  MasterServer();
  ~MasterServer();


  /**
   * Send Procam's IP to master node.
   */
  bool ping();
};

}}
