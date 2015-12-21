# This file is part of the Group 13 Project.
# Licensing information can be found in the LICENSE file.
# (C) 2015 Group 13. All rights reserved.

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(DerpVision)

# C++ flags.
ADD_COMPILE_OPTIONS(-std=c++1y)
ADD_COMPILE_OPTIONS(-D thread_local=__thread)
ADD_COMPILE_OPTIONS(-Wall)
ADD_COMPILE_OPTIONS(-Werror)
ADD_COMPILE_OPTIONS(-Wextra)
ADD_COMPILE_OPTIONS(-pthread)
ADD_COMPILE_OPTIONS(-Wmissing-declarations)
IF (CMAKE_COMPILER_IS_GNUCXX)
  SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pthread")
ENDIF()

# Macro definitions
ADD_DEFINITIONS(-DGTEST_HAS_TR1_TUPLE=0)

# Dynamic Analysis.
IF (CMAKE_BUILD_TYPE STREQUAL "Debug")
  ADD_COMPILE_OPTIONS(-fno-omit-frame-pointer)
  ADD_COMPILE_OPTIONS(-fno-optimize-sibling-calls)
  ADD_COMPILE_OPTIONS(-fsanitize=address)
  ADD_COMPILE_OPTIONS(-O1)
  ADD_COMPILE_OPTIONS(-g)
  SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=address")
  IF (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    ADD_COMPILE_OPTIONS(-fsanitize=thread)
    SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=thread")
  ENDIF()
ELSE()
  ADD_COMPILE_OPTIONS(-O3)
ENDIF()
# Clear Build type.
unset(CMAKE_BUILD_TYPE CACHE)

# Create a __FILENAME__ macro that only contains a relative path.
SET(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -D__FILENAME__='\"$(subst ${CMAKE_SOURCE_DIR}/,,$(abspath $<))\"'"
)

# Enable unit testing.
ENABLE_TESTING()

# Configure boost.
SET(Boost_USE_STATIC_LIBS        ON)
SET(Boost_USE_MULTITHREADED      ON)
SET(Boost_USE_STATIC_RUNTIME    OFF)

# Find packages using CMake.
FIND_PACKAGE(Boost REQUIRED COMPONENTS program_options filesystem system)
FIND_PACKAGE(freenect2 REQUIRED)
FIND_PACKAGE(GLUT REQUIRED)
FIND_PACKAGE(GTest REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(OpenGL REQUIRED)

# Set up include directories.
INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR})
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR})
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${freenect2_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${GLUT_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${GTEST_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${OpenGL_INCLUDE_DIRS})

# Collect the set of libraries required to link with OpenGL.
IF (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    SET(GL_LIBRARIES
        dl
        glfw3
        X11
        Xcursor
        Xrandr
        Xi
        Xinerama
        Xxf86vm
        ${GLUT_LIBRARY}
        ${OPENGL_LIBRARIES}
    )
ELSE()
    SET(GL_LIBRARIES
        glfw3
        ${GLUT_LIBRARY}
        ${OPENGL_LIBRARIES}
    )
ENDIF()

# Build everything in subfolders.
ADD_SUBDIRECTORY(core)
ADD_SUBDIRECTORY(procam)
ADD_SUBDIRECTORY(master)
ADD_SUBDIRECTORY(test)
