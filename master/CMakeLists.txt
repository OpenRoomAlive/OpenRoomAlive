# This file is part of the Group 13 Project.
# Licensing information can be found in the LICENSE file.
# (C) 2015 Group 13. All rights reserved.

SET(SOURCES
    Calibrator.cc
    MasterApplication.cc
    MasterConnectionHandler.cc
    MasterServer.cc
    ProCam.cc
    ProCamSystem.cc
)
SET(HEADERS
    Calibrator.h
    MasterApplication.h
    MasterConnectionHandler.h
    MasterConnectionHandler-inl.h
    MasterServer.h
    ProCam.h
    ProCamSystem.h
)
SET(TESTS
    CalibratorTest.cc
)
SET(LIBS
    boost_program_options
    core
    pthread
    ${OpenCV_LIBRARIES}
)

ADD_LIBRARY(master_lib
    ${SOURCES}
    ${HEADERS}
)

# Master executable.
ADD_EXECUTABLE(master
    ${SOURCES}
    ${HEADERS}
    main.cc
)
TARGET_LINK_LIBRARIES(master
    master_lib
    ${LIBS}
)

# Tests for the master library.
ADD_EXECUTABLE(master_test
    ${TESTS}
)
TARGET_LINK_LIBRARIES(master_test
    master_lib
    ${LIBS}
    ${GTEST_BOTH_LIBRARIES}
)
ADD_TEST(MasterTests master_test)
