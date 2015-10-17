# This file is part of the Group 13 Project.
# Licensing information can be found in the LICENSE file.
# (C) 2015 Group 13. All rights reserved.

SET(SOURCES
    main.cc
    GLDisplay.cc
    ProCamApplication.cc
    ProCamServer.cc
    RGBDCamera.cc
)

SET(HEADERS
    GLDisplay.h
    ProCamApplication.h
    ProCamServer.h
    RGBDCamera.cc
)

SET(LIBS
    core
    glfw3
    ${GLUT_LIBRARY}
    ${OPENGL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    pthread
)

# Choose mock or kinct implementation.
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
ELSE()
    ADD_DEFINITIONS(-DMOCK_CAMERA)

    SET(SOURCES ${SOURCES}
        MockCamera.cc
    )

    SET(HEADERS ${HEADERS}
        MockCamera.h
    )

    SET(LIBS ${LIBS}
    )
ENDIF()


# Linux-specific lbraries.
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

