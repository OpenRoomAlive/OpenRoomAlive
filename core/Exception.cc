// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include "core/Exception.h"
using namespace dv;


Exception::Exception(
    const std::string &file,
    size_t line,
    const std::string &function)
  : std::exception()
  , file_(file)
  , line_(line)
  , function_(function)
  , message_("[" + file + ":" + std::to_string(line) + "]: ")
{
}

Exception::~Exception() noexcept {
}
