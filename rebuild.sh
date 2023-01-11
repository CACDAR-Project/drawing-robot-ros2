#!/bin/bash

vcs import --recursive --shallow src < drawing_robot_ros2.repos

pip install https://cdn.evilmadscientist.com/dl/ad/public/AxiDraw_API.zip --upgrade --upgrade-strategy eager

cd src
rm -r install build log
colcon build --packages-select robot_interfaces robot_controller
source install/local_setup.bash
colcon build
source install/local_setup.bash
