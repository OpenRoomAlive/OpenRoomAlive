// This file is part of the DerpVision Project.
// Licensing information can be found in the LICENSE file.
// (C) 2015 Group 13. All rights reserved.

#include <cstdlib>
#include <iostream>
#include <string>

#include <boost/program_options.hpp>

#include "core/Master.h"
#include "core/ProCam.h"
#include "procam/ProCamApplication.h"

using namespace dv::procam;

/**
 * Null output for thrift logging.
 */
static void nullOutput(const char *message) {
  (void) message;
}

/**
 * Entry point of the application.
 */
int main(int argc, char **argv) {
  try {
    namespace po = boost::program_options;

    // Silence silly thrift output.
    apache::thrift::GlobalOutput.setOutputFunction(nullOutput);

    // Set up the description of all command line options.
    po::options_description description("DerpVision Procam Server");
    description.add_options()
        ( "help"
        , "Print this message."
        )
        ( "ip"
        , po::value<std::string>()->default_value("localhost")
        , "Set the IP of the master node."
        )
        ( "port"
        , po::value<uint16_t>()->default_value(11630)
        , "Set the port on which Procam messages master node."
        )
        ( "enable-display"
        , po::value<bool>()->default_value(true)
        , "Enable projector output."
        )
        ( "enable-kinect"
        , po::value<bool>()->default_value(true)
        , "Enable kinect input."
        )
        ( "enable-master"
        , po::value<bool>()->default_value(true)
        , "Enables the connection to the master."
        )
        ( "log-level"
        , po::value<uint16_t>()->default_value(2)
        , "Set the min. importance level of messages logged for Kinect."
          "By default it is Errors & Warnings."
        )
        ( "log-filename"
        , po::value<std::string>()->default_value("./KinectLog.txt")
        , "Set the path to the Kinect log file."
        )
        ( "effective-width"
        , po::value<size_t>()->default_value(64)
        , "Set the effective display width."
        )
        ( "effective-height"
        , po::value<size_t>()->default_value(64)
        , "Set the effective display height."
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
        options["ip"].as<std::string>(),
        options["port"].as<uint16_t>(),
        options["enable-display"].as<bool>(),
        options["enable-kinect"].as<bool>(),
        options["enable-master"].as<bool>(),
        options["log-level"].as<uint16_t>(),
        options["log-filename"].as<std::string>(),
        options["effective-width"].as<size_t>(),
        options["effective-height"].as<size_t>()
    ).run();
  } catch (const std::exception &ex) {
    std::cerr << "[Exception] " << ex.what() << std::endl;
    return EXIT_FAILURE;
  } catch (...) {
    std::cerr << "[Exception] Unknown exception." << std::endl;
    return EXIT_FAILURE;
  }
}
