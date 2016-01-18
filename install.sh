#!/usr/bin/env bash

echo "\n" | ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
brew tap homebrew/science
brew tap homebrew/versions
brew update
brew install cmake boost folly opencv3 glfw3 thrift libusb jpeg-turbo
brew link --overwrite --force cmake boost folly opencv3 glfw3 thrift libusb


cd `dirname "${BASH_SOURCE[0]}"`

cd ..
rm -rf libfreenect2
git clone https://github.com/OpenKinect/libfreenect2.git
cd libfreenect2
mkdir build && cd build
cmake .. -Wno-dev
make
make install

cd `dirname "${BASH_SOURCE[0]}"`

rm -rf build
mkdir build
cd build
cmake ..
make -j4
