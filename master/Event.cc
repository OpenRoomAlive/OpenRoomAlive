// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include "master/Event.h"

using namespace dv::master;

Event::Event(
    const ConnectionID id,
    const cv::Point3f point,
    const cv::Scalar color)
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