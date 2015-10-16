// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include "master/MasterServer.h"

dv::master::MasterServer::MasterServer() {
}

dv::master::MasterServer::~MasterServer() {
}

bool dv::master::MasterServer::ping() {
  printf("Pinged by a Procam.\n");
  return true;
}
