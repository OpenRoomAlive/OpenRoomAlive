# This file is part of the Group 13 Project.
# Licensing information can be found in the LICENSE file.
# (C) 2015 Group 13. All rights reserved.

# List of all test helpers.
SET(HELPERS
    mock/MockConnectionHandler.h
    mock/MockConnectionHandler.cc
    mock/MockEnvironment.h
    mock/MockEnvironment.cc
)

# List of all tests.
SET(TESTS
    CalibratorTest.cc
    ExceptionTest.cc
    GeometryTest.cc
    GreyCodeTest.cc
    LaserDetectorTest.cc
    ProCamSystemTest.cc
    ProjectionTest.cc
)

# Tests for the core library.
ADD_EXECUTABLE(tests
    ${HELPERS}
    ${TESTS}
)
TARGET_LINK_LIBRARIES(tests
    core
    master_lib
    procam_lib
    double-conversion
    ${GTEST_LIBRARIES}
)
ADD_TEST(UnitTests tests)
