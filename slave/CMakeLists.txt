# This file is part of the Group 13 Project.
# Licensing information can be found in the LICENSE file.
# (C) 2015 Group 13. All rights reserved.

SET(SOURCES ${SOURCES}
    BaselineCapture.cc
    BGRDCamera.cc
    Display.cc
    GLDisplay.cc
    MockCamera.cc
    MockDisplay.cc
    ProCamApplication.cc
)

SET(HEADERS ${HEADERS}
    BaselineCapture.h
    BGRDCamera.h
    Display.h
    GLDisplay.h
    MockCamera.h
    MockDisplay.h
    ProCamApplication.h
)

SET(LIBS
    core
    glfw3
    ${GLUT_LIBRARY}
    ${OPENGL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    pthread
)

# Choose mock or kinect implementation.
IF (${freenect2_FOUND})
    ADD_DEFINITIONS(-DKINECT_CAMERA)

    SET(SOURCES ${SOURCES}
        KinectCamera.cc
        KinectFileLogger.cc
    )

    SET(HEADERS ${HEADERS}
        KinectCamera.h
        KinectFileLogger.h
    )

    SET(LIBS ${LIBS}
        ${freenect2_LIBRARIES}
    )
ENDIF()

# Linux-specific libraries.
IF (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    SET(LIBS ${LIBS}
        dl
        X11
        Xcursor
        Xrandr
        Xi
        Xinerama
        Xxf86vm
    )
ENDIF()

# Slave without main (unit testing).
ADD_LIBRARY(slave_lib
    ${SOURCES}
    ${HEADERS}
)
TARGET_LINK_LIBRARIES(slave_lib
    ${LIBS}
)

# Slave executable.
ADD_EXECUTABLE(slave
    main.cc
)
TARGET_LINK_LIBRARIES(slave
    slave_lib
)
