# This file is part of the Group 13 Project.
# Licensing information can be found in the LICENSE file.
# (C) 2015 Group 13. All rights reserved.

SET(SOURCES
    main.cc
    ProCamServer.cc
)

SET(HEADERS
    ProCamServer.h
)

ADD_EXECUTABLE(slave
    ${SOURCES}
    ${HEADERS}
)

TARGET_LINK_LIBRARIES(slave
    core
    pthread
    GL 
    glfw3
    ${freenect2_LIBRARY}
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
