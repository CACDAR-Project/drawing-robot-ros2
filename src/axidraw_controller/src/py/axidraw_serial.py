#!/usr/bin/env python3
from pyaxidraw import axidraw

from robot_interfaces.srv import Status, Points
from geometry_msgs.msg import Point
from std_msgs.msg import Empty

import rclpy
from rclpy.node import Node


#TODO delete this
class Test():
    def __init__():
        ad = axidraw.AxiDraw()          # Initialize class
        ad.interactive()                # Enter interactive context
        ad.options.port = "/dev/ttyACM0"
        if not ad.connect():            # Open serial port to AxiDraw;
            quit()                      #   Exit, if no connection.
        ad.options.units = 2            # set working units to mm.
        ad.options.model = 2            # set model to AxiDraw V3/A3
        # Absolute moves follow:
        ad.moveto(10, 10)               # Pen-up move to (10mm, 10mm)
        ad.lineto(20, 10)               # Pen-down move, to (20mm, 10)
        ad.moveto(0, 0)                 # Pen-up move, back to origin.
        ad.disconnect()                 # Close serial port to AxiDraw


#TODO handle serial disconnect
class AxidrawSerial(Node):
    status = {
        "serial": "not ready",
        "motion": "waiting",
    }

    def init_serial(self, port):
        self.ad = axidraw.AxiDraw()          # Initialize class
        self.ad.interactive()                # Enter interactive context
        self.ad.options.port = port
        if not self.ad.connect():            # Open serial port to AxiDraw;
            return False
        self.ad.options.units = 2            # set working units to mm.
        self.ad.options.model = 2            # set model to AxiDraw V3/A3
        self.ad.update()                     # Process changes to options
        self.status["serial"] = "ready"
        self.status["motion"] = "ready"
        return True

    def __init__(self, port="/dev/ttyACM0"):
        super().__init__('axidraw_serial')
        self.status_srv = self.create_service(Status, 'axidraw_status', self.get_status)

        while not self.init_serial(port):
            self.get_logger().error("Failed to connect to axidraw on port:{}".format(port))

        self.move_sub = self.create_subscription(Point, 'axidraw_move', move_callback)
        self.penup_sub = self.create_subscription(Empty, 'axidraw_penup', penup_callback)
        self.pendown_sub = self.create_subscription(Empty, 'axidraw_pendown', pendown_callback)
        self.path_sub = self.create_subscription(Points, 'axidraw_path', stroke_callback)

    def get_status(self, request, response):
        response.status = status.get(request.resource, "Resource '{}' not found.".format(request.resource))
        return response

    def set_busy(self):
        self["motion"] = "busy"

    def set_ready(self):
        self["motion"] = "ready"

    def move_callback(self, msg):
        self.set_busy()

        self.get_logger().info("Received move: {}".format(msg))

        self.ad.goto(msg.x,msg.y)
        self.set_ready()

    def penup_callback(self, msg):
        self.set_busy()

        self.get_logger().info("Received penup: {}".format(msg))

        self.ad.penup()
        self.set_ready()

    def pendown_callback(self, msg):
        self.set_busy()

        self.get_logger().info("Received pendown: {}".format(msg))

        self.ad.pendown()
        self.set_ready()

    def path_callback(self, msg):
        self.set_busy()

        self.get_logger().info("Received path: {}".format(msg))

        path = [ [p.x,p.y] for p in msg.points ]
        self.ad.draw_path(path)
        self.set_ready()


def main(args=None):
    rclpy.init(args=args)

    axidraw_serial = AxidrawSerial()

    rclpy.spin(axidraw_serial)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
