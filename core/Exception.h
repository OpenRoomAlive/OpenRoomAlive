// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#pragma once

#include <exception>
#include <string>


namespace dv {

class Exception : public std::exception {
 public:
  Exception(
      const std::string &file,
      size_t line,
      const std::string &function);

  virtual ~Exception() noexcept;

  /**
   * Adds a string to the message.
   */
  Exception &operator << (const char *str) {
    message_ += str;
    return *this;
  }

  Exception &operator << (const std::string &str) {
    message_ += str;
    return *this;
  }

  /**
   * Adds a number to the message (anything that has a version of to_string).
   */
  template<typename T>
  Exception &operator << (const T &arg) {
    message_ += std::to_string(arg);
    return *this;
  }

  /**
   * Returns the formatted message.
   */
  const char * what() const noexcept override {
    return message_.c_str();
  }

  size_t getLine() const { return line_; }
  const std::string &getFile() const { return file_; }
  const std::string &getFunction() const { return function_; }

 private:
  const std::string file_;
  const size_t line_;
  const std::string function_;

  /// Full, formatted error message.
  std::string message_;
};

#define EXCEPTION() Exception(__FILE__, __LINE__, __FUNCTION__)

}
