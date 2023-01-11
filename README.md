# drawing-robot-ros2

## Building

``` sh
./rebuild.sh
```
``` sh
source src/install/local_setup.bash
```

## Running
Run
``` sh
ros2 run robot_controller dummy_controller
```
or
``` sh
ros2 launch axidraw_controller axidraw_controller
```
or
``` sh
ros2 launch lite6_controller lite6_gazebo.launch.py
```
or
``` sh
ros2 launch lite6_controller lite6_real.launch.py
```
or
``` sh
ros2 launch lite6_controller lite6_real_no_gui.launch.py
```

And simultaneously (using tmux or another terminal) run
``` sh
ros2 run drawing_controller drawing_controller src/draw_svg/svg/test.svg
```
## Docker
### Build container

``` sh
bash .docker/build.bash
```

### Run built container
``` sh
bash .docker/run.bash
```

If active changes are being made, run:
``` sh
bash .docker/devel.bash
```
This will mount the host `drawing-robot-ros2` directory in the container at `src/drawing-robot-ros2`.
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
``` sh
adduser ubuntu dialout #give access to serial devices (axidraw)
``` 

### Misc commands
``` sh
apt update
apt install git tmux python3-colcon-ros python3-pip ros-humble-moveit
``` 

``` sh
  apt install colcon
  apt search colcon
  apt install ros-dev-tools
  vi /etc/issue
  systemctl stop wpa_supplicant
  wpa_supplicant -B -i wlan0 -c /etc/wpa_supplicant/wpa_supplicant.conf
  wpa_cli
  ip link set wlan0 up
  wpa_cli
  dhclient wlan0
  ping google.com
  apt install ros-galactic-moveit
  apt install xauth
  vim /etc/ssh/sshd_config
  systemctl restart sshd
  colcon build --packages-select robot_interfaces robot_controller
```

sets priority for wlan0; uses it as gateway if connected.
/etc/netplan/50-cloud-init.yaml
``` sh
network:
    wifis:
        wlan0:
            dhcp4: true
            dhcp4-overrides:
              route-metric: 100
            optional: true
            access-points:
              "SSID":
                password: "PSK"
    ethernets:
        eth0:
            dhcp4: true
            dhcp4-overrides:
              route-metric: 200
            optional: true
    version: 2
```
/etc/ssh/sshd:
```
X11Forwarding yes
X11UseLocalhost no
```

### Access xarm webUI from different network
If connected to the pi on 192.168.22.199, one can forward the webUI to localhost:8080 with the following:
``` sh
ssh -L 8080:192.168.1.150:18333 ubuntu@192.168.22.199
```

## Moveit2 docs

``` sh

git clone https://github.com/ros-planning/moveit2.git
cd moveit2
git checkout humble

sudo apt-get install doxygen graphviz

DOXYGEN_OUTPUT_DIRECTORY=docs doxygen

firefox docs/index.html

```
