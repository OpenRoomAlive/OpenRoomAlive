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
    Compression.cc
    Conv.cc
    Exception.cc
    Geometry.cc
    GLViewer.cc
    GrayCode.cc
    Projection.cc
    Types.cc
)

# List of all files in core.
SET(HEADERS
    Async.h
    Compression.h
    Conv.h
    Exception.h
    Geometry.h
    GL.h
    GLViewer.h
    GrayCode.h
    Projection.h
    Types.h
)

# Shared core library.
ADD_LIBRARY(core
    ${SOURCES}
    ${HEADERS}
    ${THRIFT}
)
TARGET_LINK_LIBRARIES(core
    thrift
    boost_iostreams
    dl
    ${OpenCV_LIBRARIES}
)
