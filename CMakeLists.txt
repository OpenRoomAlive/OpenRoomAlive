# This file is part of the Group 13 Project.
# Licensing information can be found in the LICENSE file.
# (C) 2015 Group 13. All rights reserved.

CMAKE_MINIMUM_REQUIRED(VERSION 3.0)
PROJECT(DerpVision)

# C++ flags.
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}\
    -std=c++14 \
    -Wall      \
    -Werror    \
    -Wextra    \
    -fno-rtti"
)

# Set up include directories.
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR})
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR})

# Build everything in subfolders.
ADD_SUBDIRECTORY(core)
ADD_SUBDIRECTORY(tools)
ADD_SUBDIRECTORY(slave)
ADD_SUBDIRECTORY(master)

