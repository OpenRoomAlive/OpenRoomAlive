# This file is part of the Group 13 Project.
# Licensing information can be found in the LICENSE file.
# (C) 2015 Group 13. All rights reserved.

SET(SOURCES ${SOURCES}
    main.cc
    Display.cc
    GLDisplay.cc
    GrayCode.cc
    MockCamera.cc
    MockDisplay.cc
    ProCamApplication.cc
    ProCamServer.cc
    RGBDCamera.cc
)

SET(HEADERS ${HEADERS}
    Display.h
    GLDisplay.h
    GrayCode.h
    MockCamera.h
    MockDisplay.h
    ProCamApplication.h
    ProCamServer.h
    RGBDCamera.h
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
    )

    SET(HEADERS ${HEADERS}
        KinectCamera.h
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

ADD_EXECUTABLE(slave
    ${SOURCES}
    ${HEADERS}
)

TARGET_LINK_LIBRARIES(slave
    ${LIBS}
)
