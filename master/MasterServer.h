// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include "core/Master.h"
#include "core/Types.h"


namespace dv { namespace master {

class EventStream;

class MasterServer : virtual public MasterIf {
 public:
  MasterServer(
      const std::shared_ptr<EventStream>& stream,
      const ConnectionID id);
  ~MasterServer();

  /**
   * Send Procam's IP to master node.
   */
  bool ping() override;

  /**
   * Sends a new laser position to master.
   */
  void detectedLaser(const Point& point, const Color &color) override;

 private:
  /// Stream of events sent by ProCams for processing.
  const std::shared_ptr<EventStream> stream_;
  /// ID of the ProCam for which this handler was created.
  const ConnectionID id_;
};

}}
