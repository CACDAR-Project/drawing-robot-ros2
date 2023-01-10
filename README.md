# drawing-robot-ros2


``` sh
source install/local_setup.bash
cd src/ign_moveit2_examples/src/draw_svg/
rosdep install --from-paths . --ignore-src -r -y
colcon build
source install/local_setup.bash
ros2 launch draw_svg draw_svg.launch.py
```

## xArm lite6
- web interface: http://192.168.1.150:18333

free drive mode: https://github.com/xArm-Developer/xarm_ros#6-mode-change
## ROS2 rpi4
https://github.com/ros-realtime/ros-realtime-rpi4-image/releases

After unpacking the tar file, flash it to sd card.
Log in with "ubuntu:ubuntu".

``` sh
sudo -i
loadkeys fi
passwd ubuntu #change from default 'ubuntu' to '1234'
apt-mark hold $(uname -r) linux-firmware u-boot-rpi u-boot-tools #prevent kernel updates
apt-mark hold libraspberrypi-bin libraspberrypi-dev libraspberrypi-doc libraspberrypi0
apt-mark hold raspberrypi-bootloader raspberrypi-kernel raspberrypi-kernel-headers

apt update && apt upgrade


apt install ros-dev-tools
```


### Access xarm webUI from different network
If connected to the pi on 192.168.22.199, one can forward the webUI to localhost:8080 with the following:
``` sh
ssh -L 8080:192.168.1.150:18333 ubuntu@192.168.22.199
```
