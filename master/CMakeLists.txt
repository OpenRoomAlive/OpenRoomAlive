# This file is part of the Group 13 Project.
# Licensing information can be found in the LICENSE file.
# (C) 2015 Group 13. All rights reserved.

SET(SOURCES
    main.cc
    MasterApplication.cc
    MasterServer.cc
    MasterServerFactory.cc
    ProCam.cc
    ProCamSystem.cc
)
SET(HEADERS
    MasterApplication.h
    MasterServer.h
    MasterServerFactory.h
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

