// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <climits>

#include "master/Event.h"

using namespace dv::master;

Event::Event() 
  : id_(ULONG_MAX)
  , point_(Point())
  , color_(0)
{
}

Event::Event(const ConnectionID id, const Point point, const int64_t color)
  : id_(id)
  , point_(point)
  , color_(color)
{
  (void) point_;
  (void) id_;
  (void) color_;
}

Event::~Event() {
}