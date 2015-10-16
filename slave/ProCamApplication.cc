// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <thread>

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
    // Interfacing with master node
    std::thread networking([this]() {
      talkToMaster();
    });

    // Projector

    GLFWwindow* window;

    if (!glfwInit()) {
      return EXIT_FAILURE;
    }

    window = glfwCreateWindow(640, 480, "Hello World", NULL, NULL);
    if (!window) {
      glfwTerminate();
      return EXIT_FAILURE;
    }

    glfwMakeContextCurrent(window);

    while (!glfwWindowShouldClose(window)) {
      glClear(GL_COLOR_BUFFER_BIT);
      glfwSwapBuffers(window);
      glfwPollEvents();
    }

    glfwTerminate();


    return EXIT_SUCCESS;
  }

void ProCamApplication::talkToMaster() {
    // ONLY A BEGINNING OF NEWORK SETUP. DON'T BOTHER REVIEWING
    namespace at  = apache::thrift;
    namespace atp = apache::thrift::protocol;
    namespace att = apache::thrift::transport;

    auto socket    = boost::make_shared<att::TSocket>(masterIP_, masterPort_);
    auto transport = boost::make_shared<att::TBufferedTransport>(socket);
    auto protocol  = boost::make_shared<atp::TBinaryProtocol>(transport);
    MasterClient client(protocol);
  }

