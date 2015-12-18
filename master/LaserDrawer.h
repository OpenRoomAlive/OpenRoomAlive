// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <vector>

#include "master/ConnectionHandler.h"
#include "master/ProCamSystem.h"


namespace dv { namespace master {

class LaserDrawer {
 public:
  LaserDrawer(
      const std::vector<ConnectionID>& ids,
      const std::shared_ptr<ProCamSystem> system,
      const std::shared_ptr<EventStream> stream,
      const boost::shared_ptr<ConnectionHandler> connectionHandler);
  ~LaserDrawer();

  /**
   * Fetches events from the stream and sends appropriate drawing commands.
   */
  void run();

 private:
  /// ProCam ids.
  const std::vector<ConnectionID> ids_;
  /// ProCam system.
  const std::shared_ptr<ProCamSystem> system_;
  /// Stream of events to process.
  const std::shared_ptr<EventStream> stream_;
  /// Connection handler.
  const boost::shared_ptr<ConnectionHandler> connectionHandler_;
};

}}

