# Drawing controller

This package handles translating an SVG image into a series of motion messages (robot_interfaces/msg/Motion.msg) and sending them to a robot_controller.

## Coordinates
The origin (0,0,0) of the motion message coordinates is the top left corner of an A4 sheet with the pen in contact with the page, (1,1,0) is the bottom right corner.

Raising the pen from the page is indicated with positive z-axis values. (0,0,1) raises the pen 210mm above the paper

Negative z-values push the pen deeper into the page (if possible on the robot).
