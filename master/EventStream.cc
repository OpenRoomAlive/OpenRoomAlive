// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include "master/EventStream.h"

using namespace dv::master;

EventStream::EventStream() {
}

EventStream::~EventStream() {
}

void EventStream::push(Event event) {
  std::unique_lock<std::mutex> locker(streamLock_);
  stream_.push(event);
  streamLock_.unlock();
  sizeCond_.notify_all();
}

Event EventStream::poll() {
  std::unique_lock<std::mutex> locker(streamLock_);
  sizeCond_.wait(locker, [this]() {
    return !stream_.empty();
  });
  auto event = stream_.front();
  stream_.pop();
  return event;
}