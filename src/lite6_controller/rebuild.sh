rm -r build/ install/ log/
rosdep install --from-paths . --ignore-src -r -y
colcon build
vcs import --recursive --shallow . < lite6_controller.repos
rosdep update
apt-get update
rosdep install -y -r -i
#ros2 launch draw_svg draw_svg.launch.py
