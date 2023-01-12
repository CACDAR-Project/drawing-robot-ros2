#!/bin/bash

mkdir import
vcs import --recursive import < drawing_robot_ros2.repos
sudo rosdep init
rosdep update
rosdep install -y -r -i --rosdistro "humble" --from-paths import
source "/opt/ros/humble/setup.bash"

pip install https://cdn.evilmadscientist.com/dl/ad/public/AxiDraw_API.zip --upgrade --upgrade-strategy eager

rosdep install -y -r -i --rosdistro "humble" --from-paths src
cd src
rm -r install build log
#colcon build --merge-install --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release" && \
#colcon build --packages-select robot_interfaces robot_controller
colcon build --paths robot_interfaces robot_controller
source install/local_setup.bash
colcon build
source install/local_setup.bash
