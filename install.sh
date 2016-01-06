#!/usr/bin/env bash

brew install boost folly opencv3 glfw3 thrift

cd `dirname "${BASH_SOURCE[0]}"`

rm -rf build
mkdir build
cd build
cmake ..
make