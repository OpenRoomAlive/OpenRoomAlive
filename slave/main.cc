// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include <boost/make_shared.hpp>
#include <boost/program_options.hpp>

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
#include "core/ProCam.h"

using namespace dv;



/**
 * Encapsulates most of the functionality of the application.
 */
class ProCamApplication {
 public:
  ProCamApplication(const std::string &masterIP, uint16_t masterPort)
    : masterIP_(masterIP),
      masterPort_(masterPort),
      runProcam_(true)
  {
    (void) masterIP_;
    (void) masterPort_;
    (void) runProcam_;
  }

  int Run() {
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

 private:
  // Interfacing with master node.
  void talkToMaster() {
    // ONLY A BEGINNING OF NEWORK SETUP. DON'T BOTHER REVIEWING
    namespace at  = apache::thrift;
    namespace atp = apache::thrift::protocol;
    namespace att = apache::thrift::transport;

    auto socket    = boost::make_shared<att::TSocket>(masterIP_, masterPort_);
    auto transport = boost::make_shared<att::TBufferedTransport>(socket);
    auto protocol  = boost::make_shared<atp::TBinaryProtocol>(transport);
    MasterClient client(protocol);
  }

 private:
  /// IP of the master node.
  const std::string masterIP_;
  /// Port on which Procam messages master node.
  const uint16_t masterPort_;
  /// Flag for threads to message each other when Procam is to be shut down.
  bool runProcam_;
};



int main(int argc, char **argv) {
  try {
    namespace po = boost::program_options;

    // Set up the description of all command line options.
    po::options_description description("DerpVision Procam Server");
    description.add_options()
        ( "help"
        , "Print this message."
        )
        ( "--ip"
        , po::value<std::string>()->default_value("localhost")
        , "Set the IP of the master node."
        )
        ( "--port"
        , po::value<uint16_t>()->default_value(11630)
        , "Set the port on which Procam messages master node."
        );


    // Parse options.
    po::variables_map options;
    po::store(po::parse_command_line(argc, argv, description), options);
    po::notify(options);

    // Print help if requested.
    if (options.count("help")) {
      std::cout << description << std::endl;
      return EXIT_SUCCESS;
    }

    // Create & run the app.
    return ProCamApplication(
        options["--ip"].as<std::string>(),
        options["--port"].as<uint16_t>()
    ).Run();
  } catch (const std::exception &ex) {
    std::cerr << "[Exception] " << ex.what() << std::endl;
    return EXIT_FAILURE;
  } catch (...) {
    std::cerr << "[Exception] Unknown exception." << std::endl;
    return EXIT_FAILURE;
  }
}

