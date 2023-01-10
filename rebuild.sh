#!/bin/bash
cd src
rm -r install build log
colcon build --packages-select robot_interfaces robot_controller
source install/local_setup.bash
colcon build
source install/local_setup.bash
