# drawing-robot-ros2

This repository contains ROS2 packages which make up a system used for drawing SVG images on different robots.
These packages are in 'src/'.

Documentation and build scripts for the entire project are at the top level.

The simplest way to run the project currently is by building and running the docker container.

## Docker
### Build container

``` sh
bash .docker/build.bash
```

If build fails, consider clearing build cache.
Do not run this if you have other docker containers that you care about on your computer.
``` sh
podman builder prune --all --force
```
or 
``` sh
docker builder prune --all --force
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
If using podman instead of docker using the following will allow the container to access `/dev/` which is needed by the axidraw robot. 
``` sh
sudo bash .docker/build.bash
```
``` sh
sudo bash .docker/devel.bash
```

## TODO Building locally

Requirements:
- python3-pip 
- python3-pil.imagetk
- ros-humble-moveit
- ros-humble-ros-gz
- ignition-fortress

``` sh
./rebuild.sh
```
``` sh
source src/install/local_setup.bash
```

## Running
### RobotController
One of the following RobotControllers should be started:

DummyController echoes Motion messages to the terminal.
``` sh
ros2 run robot_controller dummy_controller
```

AxidrawController draws on the axidraw robot.
Find the serial device in "/dev/", it is usually named "/dev/ttyACMX" where X is usually 0.
Try a different serial port if the axidraw_controller continuously logs a message about failing to connect.
``` sh
ros2 launch axidraw_controller axidraw_controller.launch.py serial_port:=/dev/ttyACM0
```

This starts the simulated lite6
``` sh
ros2 launch lite6_controller lite6_gazebo.launch.py
```

This runs the real lite6
``` sh
ros2 launch lite6_controller lite6_real.launch.py
```

This runs the real lite6 without Rviz (can be run on headless device over ssh)
``` sh
ros2 launch lite6_controller lite6_real_no_gui.launch.py
```

### DrawingController
Once a RobotController is running, simultaneously (using tmux or another terminal) run
``` sh
ros2 run drawing_controller drawing_controller svg/test.svg
```
This will draw the svg image given as the last argument.

### tmux workflow
lite6 interface: http://192.168.1.150:18333

#### Raspberry pi
On the raspberry pi run 
``` sh
./setup_ros.sh
```
This will open a tmux session with the necessary ros2 packages sourced.
#### Docker container
``` sh
tmux
```

If actively


## SVG compatibility info
Tested with SVG from the following programs
- Inkscape
- Inkpad
- Affinitydraw
- vtracer

Delimiter characters seem to vary somewhat.
The following examples work:
TODO ADD EXAMPLES OF SVG PATHS

Make sure that all shapes in the SVG are within the bounds defined by height and width (or viewbox).
Shapes outside of bounds will cause the robot to frequently visit the top left corner and edges of the paper and not draw the desired image.

The following SVG primitives are supported:
| Primitive                           | Support  |
|-------------------------------------|----------|
| a                                  | no       |
| animate                            | no       |
| animateMotion                      | no       |
| animateTransform                   | no       |
| circle                             | no       |
| clipPath                           | no       |
| defs                               | no       |
| desc                               | no       |
| discard                            | no       |
| ellipse                            | no       |
| feBlend                            | no       |
| feColorMatrix                      | no       |
| feComponentTransfer                | no       |
| feComposite                        | no       |
| feConvolveMatrix                   | no       |
| feDiffuseLighting                  | no       |
| feDisplacementMap                  | no       |
| feDistantLight                     | no       |
| feDropShadow                       | no       |
| feFlood                            | no       |
| feFuncA                            | no       |
| feFuncB                            | no       |
| feFuncG                            | no       |
| feFuncR                            | no       |
| feGaussianBlur                     | no       |
| feImage                            | no       |
| feMerge                            | no       |
| feMergeNode                        | no       |
| feMorphology                       | no       |
| feOffset                           | no       |
| fePointLight                       | no       |
| feSpecularLighting                 | no       |
| feSpotLight                        | no       |
| feTile                             | no       |
| feTurbulence                       | no       |
| filter                             | no       |
| foreignObject                      | no       |
| g                                  | yes      |
| hatch                              | no       |
| hatchpath                          | no       |
| image                              | no       |
| line                               | yes      |
| linearGradient                     | no       |
| marker                             | no       |
| mask                               | no       |
| metadata                           | no       |
| mpath                              | no       |
| path                               | partial  |
| pattern                            | no       |
| polygon                            | yes      |
| polyline                           | yes      |
| radialGradient                     | no       |
| rect                               | no       |
| script                             | no       |
| set                                | no       |
| stop                               | no       |
| style                              | no       |
| svg                                | no       |
| switch                             | no       |
| symbol                             | no       |
| text                               | no       |
| textPath                           | no       |
| title                              | no       |
| tspan                              | no       |
| use                                | no       |
| view                               | no       |

And the following SVG path commands are supported:
| Command type           | Supported         | Unsupported |
|------------------------|-------------------|-------------|
| MoveTo                 |  M, m             |             |
| LineTo                 |  L, l, H, h, V, v |             |
| Cubic Bézier Curve     |  C, c, S, s       |             |
| Quadratic Bézier Curve |  Q, q             | T, t        |
| Elliptical Arc Curve   |                   | A, a        |
| ClosePath              |  Z, z             |             |


## Axidraw concerns
## xArm concerns
TODO make TCP height diagram

The following paths work, notic

## Creating compatible SVG images
https://github.com/visioncortex/vtracer

Use single layer (g) SVGs

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
