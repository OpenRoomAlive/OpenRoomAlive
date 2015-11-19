// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include "core/Types.h"

namespace dv {

std::ostream& operator << (std::ostream& os, const Size& res) {
  os << res.width << "x" << res.height;
  return os;
}

std::istream& operator >> (std::istream& is, Size &size) {
  char x;
  if (!(is >> size.width >> x >> size.height) || x != 'x') {
    is.setstate(std::ios_base::failbit);
  }
  return is;
}

}
