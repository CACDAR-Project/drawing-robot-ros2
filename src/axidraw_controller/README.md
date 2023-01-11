# axidraw controller

Implements robot_controller for the Axidraw robot.

`axidraw_serial.py` is used to communicate with the robot using the python API.
If more direct control is desired, this can be implemented by sending serial commands directly to the [EBB control board](http://evil-mad.github.io/EggBot/ebb.html) of the Axidraw.

On linux systems the board appears on `/dev/ttyACM0`.
