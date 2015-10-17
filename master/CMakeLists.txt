# This file is part of the Group 13 Project.
# Licensing information can be found in the LICENSE file.
# (C) 2015 Group 13. All rights reserved.

SET(SOURCES
    main.cc
    MasterServer.cc
    ProCam.cc
    ProCamSystem.cc
)
SET(HEADERS
    MasterServer.h
    ProCam.h
    ProCamSystem.h
)

ADD_EXECUTABLE(master
    ${SOURCES}
    ${HEADERS}
)
TARGET_LINK_LIBRARIES(master
    boost_program_options
    core
    pthread
)

