#!/bin/bash

PACKAGE_SRC_PATH=~/dev_ws/log/latest_build/can_interface
PACKAGE_INSTALL_PATH=~/dev_ws/install/can_interface
PACKAGE_BUILD_PATH=~/dev_ws/build/can_interface

CMAKE_PREFIX_PATH=/opt/ros/humble /usr/bin/cmake $PACKAGE_SRC_PATH -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja -DCMAKE_INSTALL_PREFIX=$PACKAGE_INSTALL_PATH
CMAKE_PREFIX_PATH=/opt/ros/humble /usr/bin/cmake --build $PACKAGE_BUILD_PATH -- -j16 -l16
CMAKE_PREFIX_PATH=/opt/ros/humble /usr/bin/cmake --install $PACKAGE_BUILD_PATH

cp $PACKAGE_BUILD_PATH/compile_commands.json $PACKAGE_BUILD_PATH/../
