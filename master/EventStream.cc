// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include "master/EventStream.h"

using namespace dv::master;

EventStream::EventStream()
  : closedStream_(false)
{
}

EventStream::~EventStream() {
}

void EventStream::push(Event event) {
  std::unique_lock<std::mutex> locker(streamLock_);

  stream_.push(event);
  streamLock_.unlock();
  sizeCond_.notify_all();
}

std::pair<bool, Event> EventStream::poll() {
  std::unique_lock<std::mutex> locker(streamLock_);
  // Wait until stream is closed or stream is non-empty.
  sizeCond_.wait(locker, [this]() {
    return closedStream_ || !stream_.empty();
  });

  // If stream closed, it may be empty.
  if (closedStream_) {
    return std::make_pair(false,
        Event(0, cv::Point3f(), cv::Scalar()));
  }

  auto eventPair = std::make_pair(true, stream_.front());
  stream_.pop();
  return eventPair;
}

void EventStream::close() {
  closedStream_ = true;
  sizeCond_.notify_all();
}