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
    ${freenect2_LIBRARY}
)
