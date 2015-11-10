// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <iostream>

#include "master/Event.h"
#include "master/MasterServer.h"

using namespace dv::master;

MasterServer::MasterServer(
    const std::shared_ptr<EventStream>& stream,
    const ConnectionID id)
  : stream_(stream) 
  , id_(id)
{
}

MasterServer::~MasterServer() {
}

bool MasterServer::ping() {
  return true;
}

void MasterServer::detectedLaser(const Point& point, const int64_t color) {
  stream_->push(Event(id_, point, color));
}
