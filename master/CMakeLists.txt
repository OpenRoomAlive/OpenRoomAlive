# This file is part of the Group 13 Project.
# Licensing information can be found in the LICENSE file.
# (C) 2015 Group 13. All rights reserved.

SET(SOURCES
    Calibrator.cc
    ConnectionHandler.cc
    Event.cc
    EventStream.cc
    LaserDrawer.cc
    MasterApplication.cc
    MasterConnectionHandler.cc
    MasterServer.cc
    ProCam.cc
    ProCamRecorder.cc
    ProCamSystem.cc
    RecordingConnectionHandler.cc
)
SET(HEADERS
    Calibrator.h
    ConnectionHandler.h
    Event.h
    EventStream.h
    LaserDrawer.h
    MasterApplication.h
    MasterConnectionHandler.h
    MasterConnectionHandler-inl.h
    MasterServer.h
    ProCam.h
    ProCamRecorder.h
    ProCamSystem.h
    RecordingConnectionHandler.h
)

# Master library (unit testing).
ADD_LIBRARY(master_lib
    ${SOURCES}
    ${HEADERS}
)
TARGET_LINK_LIBRARIES(master_lib
    core
    folly
    glog
    pthread
    double-conversion
    ${GL_LIBRARIES}
    ${Boost_LIBRARIES}
    ${freenect2_LIBRARIES}
    dl
)

# Master executable.
ADD_EXECUTABLE(master
    main.cc
)
TARGET_LINK_LIBRARIES(master
    master_lib
)
