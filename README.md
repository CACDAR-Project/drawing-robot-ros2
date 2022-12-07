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
