OpenRoomAlive
===
Setup Guide
---
OSX (El Capitan)
---
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

Linux (Ubuntu 15.10)
---
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

DocSoc Developer FAQ
---
1. `arc install-certificate` failed to connect to server due to SSL error.

        Export self-signed SSL certificate from Chrome and save it as ./libphutil/resources/ssl/custom.pem

2. How do I setup `arc` on lab machines?

        git clone https://github.com/phacility/arcanist.git
        git clone https://github.com/phacility/libphutil.git
        echo "set path = ($path ~/work/arcanist/bin)" >> ~/.cshrc
        source ~/.cshrc
        cd ~/work/DerpVision
        arc which
