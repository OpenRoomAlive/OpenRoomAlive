// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <queue>

#include "master/Event.h"

namespace dv { namespace master {

/**
 * Class encapsulating the queue of events sent by ProCams and synchronisation
 * mechanisms.
 */
class EventStream {
 public:
  EventStream();
  ~EventStream();

  /**
   * Inserts an event at the end of the stream.
   */
  void push(Event event);

  /**
   * Fetches the oldest event from the stream if stream is not closed/gets
   * closed while waiting for an Event - returns <false, Event> if no Event
   * was fetched and processing should be stopped.
   */
  std::pair<bool, Event> poll();

  /**
   * Closes the stream.
   */
  void close();

 private:
  /// Stream of events.
  std::queue<Event> stream_;
  /// Condition variable waiting on stream to be non-empty.
  std::condition_variable sizeCond_;
  /// Mutex protecting the stream.
  std::mutex streamLock_;
  /// Bool representing the state of the stream.
  std::atomic<bool> closedStream_;
};

} }
