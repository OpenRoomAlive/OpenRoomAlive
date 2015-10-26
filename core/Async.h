// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <exception>
#include <future>
#include <mutex>
#include <thread>


namespace dv {

/**
 * Specialization for void functions.
 */
std::future<void> asyncExecute(std::function<void()> func) {
  // Lambda to wrap the task & fulfill the promise.
  auto executor = [&] (std::promise<void> promise) {
    try {
      func();
      promise.set_value();
    } catch (...) {
      promise.set_exception(std::current_exception());
    }
  };

  // Create the promise & the future.
  std::promise<void> promise;
  auto future = promise.get_future();

  // Launch the thread and detach it so the destructor does not terminate.
  std::thread(executor, std::move(promise)).detach();

  return future;
}

}
