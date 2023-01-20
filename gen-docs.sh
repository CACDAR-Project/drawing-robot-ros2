#!/usr/bin/env sh

pushd src/robot_controller/
doxygen
popd
make html
