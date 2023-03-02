#!/usr/bin/env sh
# generates docs

pushd ..

pushd src/robot_controller/
doxygen
popd

pushd src/lite6_controller/
doxygen
popd

pushd src/axidraw_controller/
doxygen
popd

make html


