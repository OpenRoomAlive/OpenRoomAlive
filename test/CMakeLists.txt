# This file is part of the Group 13 Project.
# Licensing information can be found in the LICENSE file.
# (C) 2015 Group 13. All rights reserved.

# List of all tests & mock classes.
SET(TESTS
    CalibratorTest.cc
    ExceptionTest.cc
    GeometryTest.cc
    ProjectionTest.cc
)

# Tests for the core library.
ADD_EXECUTABLE(tests
    ${TESTS}
)
TARGET_LINK_LIBRARIES(tests
    core
    master_lib
    ${GTEST_BOTH_LIBRARIES}
)
ADD_TEST(UnitTests tests)
