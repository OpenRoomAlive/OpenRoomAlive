# This file is part of the Group 13 Project.
# Licensing information can be found in the LICENSE file.
# (C) 2015 Group 13. All rights reserved.

SET(SOURCES
    main.cc
    ProCamApplication.cc
    ProCamServer.cc
)

SET(HEADERS
    ProCamApplication.h
    ProCamServer.h
)

ADD_EXECUTABLE(slave
    ${SOURCES}
    ${HEADERS}
)

TARGET_LINK_LIBRARIES(slave
    core
    glfw3
    ${freenect2_LIBRARY}
    ${GLUT_LIBRARY}
    ${OPENGL_LIBRARIES}
    pthread
)

IF (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    TARGET_LINK_LIBRARIES(slave
        dl
        X11
        Xcursor
        Xrandr
        Xi
        Xinerama
        Xxf86vm
)
ENDIF()
