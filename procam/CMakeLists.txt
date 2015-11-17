# This file is part of the Group 13 Project.
# Licensing information can be found in the LICENSE file.
# (C) 2015 Group 13. All rights reserved.

SET(SOURCES ${SOURCES}
    BaselineCapture.cc
    BGRDCamera.cc
    Display.cc
    GLDisplay.cc
    KinectCamera.cc
    KinectFileLogger.cc
    MockCamera.cc
    MockDisplay.cc
    ProCamApplication.cc
)

SET(HEADERS ${HEADERS}
    BaselineCapture.h
    BGRDCamera.h
    Display.h
    GLDisplay.h
    KinectCamera.h
    KinectFileLogger.h
    MockCamera.h
    MockDisplay.h
    ProCamApplication.h
)

# ProCam without main (unit testing).
ADD_LIBRARY(procam_lib
    ${SOURCES}
    ${HEADERS}
)
TARGET_LINK_LIBRARIES(procam_lib
    core
    pthread
    ${EXTRA_LIBS}
    ${freenect2_LIBRARIES}
    ${GL_LIBS}
    ${OpenCV_LIBRARIES}
)

# Procam executable.
ADD_EXECUTABLE(procam
    main.cc
)

TARGET_LINK_LIBRARIES(procam
    procam_lib
)

