# This file is part of the Group 13 Project.
# Licensing information can be found in the LICENSE file.
# (C) 2015 Group 13. All rights reserved.

# Generate Thrift Code.
SET(GEN ${CMAKE_CURRENT_BINARY_DIR})
SET(THRIFT
    ${GEN}/ProCam.cpp
    ${GEN}/ProCam.h
    ${GEN}/Master.cpp
    ${GEN}/Master.h
    ${GEN}/Protocol_constants.cpp
    ${GEN}/Protocol_constants.h
    ${GEN}/Protocol_types.cpp
    ${GEN}/Protocol_types.h
)
ADD_CUSTOM_COMMAND(
  OUTPUT
      ${THRIFT}
  COMMAND thrift
      -strict
      -out ${GEN}
      --gen cpp
      ${CMAKE_CURRENT_SOURCE_DIR}/Protocol.thrift
  MAIN_DEPENDENCY
      ${CMAKE_CURRENT_SOURCE_DIR}/Protocol.thrift
  COMMENT
      "Generating Thrift."
)
SET_SOURCE_FILES_PROPERTIES(
    ${THRIFT}
    PROPERTIES GENERATED TRUE
)

# List of all sources in core.
SET(SOURCES
    Conv.cc
    Exception.cc
    Projection.cc
    GrayCode.cc
)

# List of all files in core.
SET(HEADERS
    Async.h
    Conv.h
    Exception.h
    Projection.h
    GrayCode.h
)

# List of all tests.
SET(TESTS
    ExceptionTest.cc
)

# Shared core library.
ADD_LIBRARY(core
    ${SOURCES}
    ${HEADERS}
    ${THRIFT}
)
TARGET_LINK_LIBRARIES(core
    boost_program_options
    thrift
)

# Tests for the core library.
ADD_EXECUTABLE(core_test
    ${TESTS}
)
TARGET_LINK_LIBRARIES(core_test
    core
    ${GTEST_BOTH_LIBRARIES}
)
ADD_TEST(CoreTests core_test)

