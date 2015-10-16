// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <iostream>
#include <thread>
#include <iostream>

#include <boost/make_shared.hpp>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#include <GLFW/glfw3.h>

#include <libfreenect2/libfreenect2.hpp>

#include <thrift/protocol/TBinaryProtocol.h>
#include <thrift/transport/TSocket.h>
#include <thrift/transport/TTransportUtils.h>

#include "core/Master.h"
#include "core/Exception.h"
#include "slave/ProCamApplication.h"

using namespace dv;

ProCamApplication::ProCamApplication(
  const std::string &masterIP,
  uint16_t masterPort)
    : masterIP_(masterIP),
      masterPort_(masterPort),
      runProcam_(true)
  {
    (void) masterIP_;
    (void) masterPort_;
    (void) runProcam_;
  }

int ProCamApplication::Run() {
  // Responding to master node requests
  std::thread networking([this]() {
    respondToMaster();
  });

  // Send Procam's IP to master
  namespace at  = apache::thrift;
  namespace atp = apache::thrift::protocol;
  namespace att = apache::thrift::transport;

  auto socket    = boost::make_shared<att::TSocket>(masterIP_, masterPort_);
  auto transport = boost::make_shared<att::TBufferedTransport>(socket);
  auto protocol  = boost::make_shared<atp::TBinaryProtocol>(transport);
  MasterClient masterClient(protocol);

  try {
    transport->open();

    printf("Sending Procam's IP to master node...\n");
    if (!masterClient.ping()) {
      printf("Master node rejected IP of the Procam.\n");
      return EXIT_FAILURE;
    }
    printf("Master node accepted IP of the Procam.\n");

    transport->close();
  } catch (apache::thrift::TException& tx) {
    std::cout << "ERROR: " << tx.what() << std::endl;
  }

  // Kinect.
  // TODO(ilijar): set up the kinect.

  // Projector
  GLFWwindow *window;

  try {
    if (!glfwInit()) {
      throw EXCEPTION() << "Cannot initialize GLFW.";
    }

    int monitorCount;
    GLFWmonitor **monitors = glfwGetMonitors(&monitorCount);

    if (!monitors || monitorCount < 1) {
      throw EXCEPTION() << "No monitors found.";
    }

    GLFWmonitor *projector = monitors[monitorCount - 1];

    if (monitorCount == 1) {
      std::cerr << "Only primary monitor found.\n";
    } else if (monitorCount > 2) {
      std::cerr << "Multiple secondary monitors were found.\n";
    }

    const GLFWvidmode* mode = glfwGetVideoMode(projector);
    window = glfwCreateWindow(
        mode->width,
        mode->height,
        "OpenRoom",
        projector, NULL);

    if (!window) {
      throw EXCEPTION() << "Cannot open GLFW window.\n";
    }

    glfwMakeContextCurrent(window);
    glViewport(0, 0, mode->width, mode->height);
  } catch (...) {
    glfwTerminate();
    throw;
  }

  while (!glfwWindowShouldClose(window)) {
    glClear(GL_COLOR_BUFFER_BIT);

    glBegin(GL_QUADS);
      glColor3f(1.0f, 0.0f, 0.0f); glVertex2f(-1.0f, -1.0f);
      glColor3f(0.0f, 1.0f, 0.0f); glVertex2f(-1.0f, +1.0f);
      glColor3f(0.0f, 0.0f, 1.0f); glVertex2f(+1.0f, +1.0f);
      glColor3f(1.0f, 1.0f, 0.0f); glVertex2f(+1.0f, -1.0f);
    glEnd();

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  glfwTerminate();

  return EXIT_SUCCESS;
}

void ProCamApplication::respondToMaster() {
  // Run server responding to master node requests for data.
}

