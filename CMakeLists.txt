# This file is part of the Group 13 Project.
# Licensing information can be found in the LICENSE file.
# (C) 2015 Group 13. All rights reserved.

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(DerpVision)

# C++ flags.
SET(CMAKE_CXX_FLAGS 
    "${CMAKE_CXX_FLAGS} -std=c++1y -Wall -Werror -Wextra -fno-rtti"
)

# Find packages using CMake.
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/tools")
find_package(Freenect2 REQUIRED)

# Set up include directories.
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR})
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR})
INCLUDE_DIRECTORIES(freenect2_INCLUDE_DIR)

# Don't judge.
IF (HANQ)
  INCLUDE_DIRECTORIES(/vol/bitbucket/qh812/include)
  SET(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} /vol/bitbucket/qh812/lib)
ENDIF()

# Build everything in subfolders.
ADD_SUBDIRECTORY(core)
ADD_SUBDIRECTORY(slave)
ADD_SUBDIRECTORY(master)

