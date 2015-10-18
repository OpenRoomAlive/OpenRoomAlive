// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <gtest/gtest.h>

#include "core/Exception.h"

using namespace dv;


/**
 * Tests that the file name recorded by the exception is correct.
 */
TEST(ExceptionTest, TestFileName) {
  try {
    throw EXCEPTION() << "test";
  } catch (const Exception &ex) {
    ASSERT_EQ("core/ExceptionTest.cc", ex.getFile());
    return;
  }
  FAIL() << "Exception not thrown.";
}
