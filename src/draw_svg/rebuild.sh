rm -r build/ install/ log/
rosdep install --from-paths . --ignore-src -r -y
colcon build
#ros2 launch draw_svg draw_svg.launch.py
