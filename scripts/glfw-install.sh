#!/bin/bash

# The Ubuntu package repo has a very old version of GLFW
# It is therefore necessary to compile and install from source

git clone https://github.com/glfw/glfw.git /tmp/glfw
cd /tmp/glfw
cmake . -DBUILD_SHARED_LIBS=ON
make
sudo make install
sudo ldconfig
cd -
