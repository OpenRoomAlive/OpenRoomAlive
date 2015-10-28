// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <vector>

#include "master/MasterConnectionHandler.h"

namespace dv { namespace master {

/**
 * Class wrapping the functionality responsible for ProCam calibration.
 */
class Calibrator {
 public:
  Calibrator(
      const std::vector<ConnectionID>& ids,
      const boost::shared_ptr<MasterConnectionHandler>& connectionHandler);

  ~Calibrator();

  /**
   * Function which loops over all connections (procams) and for each
   * procam it sends requests to display a sequence of gray codes. It
   * should be run in a separate thread.
   */
  void displayGrayCodes();

 private:
  /// IDs of the procam connections
  const std::vector<ConnectionID> ids_;
  /// Pointer to the connection handler
  const boost::shared_ptr<MasterConnectionHandler> connectionHandler_;
};

} }