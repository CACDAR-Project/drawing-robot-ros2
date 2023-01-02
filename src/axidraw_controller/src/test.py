#!/usr/bin/env python3
from pyaxidraw import axidraw   # import module
ad = axidraw.AxiDraw()          # Initialize class
ad.interactive()                # Enter interactive context
ad.options.port = "/dev/ttyACM0"
if not ad.connect():            # Open serial port to AxiDraw;
    quit()                      #   Exit, if no connection.
ad.options.units = 1            # set working units to cm.
ad.options.model = 2            # set model to AxiDraw V3/A3
                                # Absolute moves follow:
ad.moveto(10, 10)               # Pen-up move to (10cm, 10cm)
ad.lineto(20, 10)               # Pen-down move, to (20cm, 10)
ad.moveto(0, 0)                 # Pen-up move, back to origin.
ad.disconnect()                 # Close serial port to AxiDraw
