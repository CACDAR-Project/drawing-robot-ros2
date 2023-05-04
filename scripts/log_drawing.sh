#!/usr/bin/env sh
# Logs drawn images
# ./log_drawing.sh ROBOTNAME SVG
# ROBOTNAME: robot name (logged before output)
# SVG: svg path to be drawn

date >> data.txt
echo $1 >> data.txt
ros2 run drawing_controller drawing_controller $2 2>&1 | grep 'Results' | tee -a data.txt
echo '\n' >> data.txt

#{'svg processing time': '00:00:02', 'failure': 65, 'total time': '00:00:09', 'drawing time': '00:00:06'}
