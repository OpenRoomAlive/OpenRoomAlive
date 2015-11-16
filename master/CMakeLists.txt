# This file is part of the Group 13 Project.
# Licensing information can be found in the LICENSE file.
# (C) 2015 Group 13. All rights reserved.

SET(SOURCES
    Calibrator.cc
    Event.cc
    EventStream.cc
    GLViewer.cc
    MasterApplication.cc
    MasterConnectionHandler.cc
    MasterServer.cc
    ProCam.cc
    ProCamSystem.cc
)
SET(HEADERS
    Calibrator.h
    Event.h
    EventStream.h
    GLViewer.h
    MasterApplication.h
    MasterConnectionHandler.h
    MasterConnectionHandler-inl.h
    MasterServer.h
    ProCam.h
    ProCamSystem.h
)

# Master library (unit testing).
ADD_LIBRARY(master_lib
    ${SOURCES}
    ${HEADERS}
)
TARGET_LINK_LIBRARIES(master_lib
    boost_program_options
    core
    pthread
    ${OpenCV_LIBRARIES}
    ${GL_LIBS}
)

# Master executable.
ADD_EXECUTABLE(master
    main.cc
)
TARGET_LINK_LIBRARIES(master
    master_lib
)
