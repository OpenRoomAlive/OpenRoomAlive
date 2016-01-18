# OpenRoomAlive

The main motivation of the OpenRoomAlive project was to build a system that,
using commodity hardware, would be able to automatically calibrate itself to any
room environment, construct a 3D model of the room and hence provide developers
with a versatile platform for building spatial augmented reality applications.
OpenRoomAlive is a cross-platform, open source implementation of the RoomAlive
Toolkit developed by Microsoft Research. It was designed to be run on any
POSIX-compliant system, without being tied to the Kinect for Windows API. We
were aiming to implement the calibration and 3D reconstruction modules and build
a small augumented reality demo (laser painting), while implementing everything
in a robust and well-tested manner.

References:
* Original project: http://research.microsoft.com/en-us/projects/roomalive/
* Original Toolkit repository: https://github.com/Kinect/RoomAliveToolkit

# Prerequisities

* Master node (one in the system)
  * Machine running OSX/Linux
* ProCam node (as many as needed to cover the surface of desired environment)
  * Machine with USB 3.0 running OSX/Linux
  * Kinect 2 camera, commodity projector
* Local network connection between machines handling the Master and ProCam nodes

# Setup Guide for Developers

### OSX (El Capitan)

1. Install libfreenect2, following the installation guide specific to your OS

		https://github.com/OpenKinect/libfreenect2

2. Install gtest

		git clone git@github.com:google/googletest.git
		cd googletest
		mkdir build
		cd build
		cmake ..
		make
		make install

3. Install project dependencies

		brew install boost folly opencv3 glfw3 thrift

4. Build using Clang

		mkdir build
		cd build
		# cmake -DCMAKE_BUILD_TYPE=Debug ..
		cmake ..
		make

### Linux (Ubuntu 15.10)

1. Install libfreenect2, following the installation guide specific to your OS

		https://github.com/OpenKinect/libfreenect2

2. Install project dependencies

		sudo apt-get install libboost-dev libopencv-dev libglfw3-dev libgtest-dev

3. Build and compile folly and thrift from source

		https://github.com/facebook/folly
		http://thrift.apache.org/docs/BuildingFromSource

4. Build using GCC

		mkdir build
		cd build
		# cmake -DCMAKE_BUILD_TYPE=Debug ..
		cmake ..
		make

## Starting the applications

An OpenRoomAlive developer has access to the full set of flags and options if
they directly use the CLI (command-line interface). The **Master Server** and
**ProCam Server** can be run from the main OpenRoomAlive directory with
**./build/master/master** and **./build/procam/procam** respectively once the
system is built and camera-projector pairs are connected to machines handling
ProCam nodes.

## All flags and options

* OpenRoomAlive Master Server

        --help                 Print this message.
        --port arg(=11630)     Set the port to listen on.
        --procam-total arg(=1) Set the number of procams expected to connect.
        --record arg           Set the path to the directory where data will be saved.
        --calibrate arg(=1)    Re-calibrates the system.
        --render arg(=0)       Renders the reconstructed 3D mesh.
        --two-step-k arg(=0)   Compute the calibration matrices in two steps.

* OpenRoomAlive ProCam Server

        --help                       Print this message.
        --ip arg(=localhost)         Set the IP of the Master node.
        --port arg(=11630)           Set the port on which ProCam messages Master node.
        --enable-display arg(=1)     Enable projector output.
        --enable-kinect arg(=1)      Enable kinect input.
        --enable-master arg(=1)      Enables the connection to the Master.
        --log-level arg(=2)          Set the min. importance level of messages logged.
        --log-filename arg           Set the path to the Kinect log file.
        --effective-size arg(=64x64) Set the effective display size.
        --latency arg(=250)          Set the projector delay.


# GUI - Non-Developers

There are also GUI wrappers available on OSX designed to offer a simplified,
i.e. not for development, version of the system. A new user needs to follow two
steps to start the software (once all the devices are connected - i.e. a
Kinect 2 and a projector are connected to each machine handling a ProCam node):
  * On each machine, double-click on **install.sh** to install dependencies and
    build the product. This may take several minutes and also might ask you for
    your password at the beginning (depending on what is already installed)
  * On each machine, double-click on either **runmaster.sh** or **runprocam.sh**
    to start the GUI wrapper for the Master node (only one) or ProCam node

When using the GUI triggered by **runmaster.sh**, the user has access to:
  * Setting the expected number of ProCam nodes/units
  * Choosing whether system needs recalibration
  * Choosing between 3D reconstruction rendering and laser painting

When using the GUI triggered by **runmprocam.sh**, the user has access to:
  * Setting the display resolution
  * Setting the IP address of the Master node


# FAQ

1. How to arrange the devices?

  In the usual setup, a Kinect 2 camera is put on top of a projector (both devices
directed the same way) and such a pair is connected to a machine handling a
ProCam node. Now the projected area by the machine's projector should overlap
(~10%) with the projected area from a different projector (i.e. from another
ProCam unit). Such a chain (or a graph) of projected areas should together
cover the desired surface area.

  Keep in mind that the area covered by each overlap region cannot be just one
planar surface - there needs to be an obstacle. Same applies to each
projected area on its own, but a proper obstacle in an overlap with a different
projected area should also work as the obstacle for the given projected area.

  Finally, there needs to be a (preferably dedicated) machine responsible for
running the Master node governing the system.

2. How to choose resolution of each projector?

  For best performance, it is preferred to use native or half of native resolution.
Also, keep in mind that pixels displayed by each projector should have similar
physical dimensions for coherent experience.

3. How to check IP of the Master node?

  You can either use the **ifconfig** command on the machine running the Master
  node or check it by visiting (on Master node's machine) e.g. http://ip4.me/.



# DocSoc Developer FAQ

1. `arc install-certificate` failed to connect to server due to SSL error.

        Export self-signed SSL certificate from Chrome and save it as ./libphutil/resources/ssl/custom.pem

2. How do I setup `arc` on lab machines?

        git clone https://github.com/phacility/arcanist.git
        git clone https://github.com/phacility/libphutil.git
        echo "set path = ($path ~/work/arcanist/bin)" >> ~/.cshrc
        source ~/.cshrc
        cd ~/work/OpenRoomAlive
        arc which
