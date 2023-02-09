#!/usr/bin/env python3
from pyaxidraw import axidraw

from robot_interfaces.srv import Status
from robot_interfaces.msg import Points
from geometry_msgs.msg import Point
from std_msgs.msg import Empty

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

#TODO handle serial disconnect

class AxidrawSerial(Node):
    """
    Class for forwarding axidraw python API functions to ROS2 topics.

    ...

    Attributes
    ----------
    status : dict
        contains the robot's status

    Methods
    -------
    init_serial(port)

    Services
    -------
    Status, 'axidraw_status'

    Topics
    -------
    Point, 'axidraw_move'
    Empty, 'axidraw_penup'
    Empty, 'axidraw_pendown'
    Points, 'axidraw_path'
    """

    status = {
        "serial": "not ready",
        "motion": "waiting",
    }

    def init_serial(self, port):
        '''
        Initiates connection to axidraw over serial.

            Parameters:
                    port (string): The serial port or path to serial device (example: "/dev/ttyACM0")
            Returns:
                    False if connection failed and True if it succeeded.
        '''
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

    def __init__(self):
        """
        Sets up a connection to the axidraw robot and starts the 'axidraw_***' services and topic subscriptions.
        Retries connection to axidraw if it fails.
        Fetches port from ROS2 parameter 'serial_port', defaulting to '/dev/ttyACM0'.

        Parameters
        ----------
        """

        super().__init__('axidraw_serial')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        port = self.get_parameter('serial_port').get_parameter_value().string_value

        self.status_srv = self.create_service(Status, 'axidraw_status', self.get_status)

        while not self.init_serial(port):
            self.get_logger().error("Failed to connect to axidraw on port:{}".format(port))

        self.move_sub = self.create_subscription(Point, 'axidraw_move', self.move_callback, qos_profile=QoSProfile(depth=1))
        self.penup_sub = self.create_subscription(Empty, 'axidraw_penup', self.penup_callback, qos_profile=QoSProfile(depth=1))
        self.pendown_sub = self.create_subscription(Empty, 'axidraw_pendown', self.pendown_callback, qos_profile=QoSProfile(depth=1))
        self.path_sub = self.create_subscription(Points, 'axidraw_path', self.stroke_callback, qos_profile=QoSProfile(depth=1))

    def get_status(self, request, response):
        '''
        Looks up the status of the requested resource, sets it as the response status and returns it.
        Used directly by the "axidraw_status" service.

            Parameters:
                    request (robot_interfaces.srv.Status.request): The request
                    response (robot_interfaces.srv.Status.response): The response
            Returns:
                    response (robot_interfaces.srv.Status.response): The response with the data added. If not found returns "Resource 'X' not found.".
        '''
        response.status = self.status.get(request.resource, "Resource '{}' not found.".format(request.resource))
        return response

    def go_home(self):
        '''
        Moves the robot to (0,0).
            Parameters:
            Returns:
        '''
        self.status["motion"] = "busy"
        if self.status["serial"] == "ready":
            self.ad.moveto(0,0)
        self.status["motion"] = "ready"

    def set_busy(self):
        '''
        Sets the robot motion to "busy"
            Parameters:
            Returns:
        '''
        self.status["motion"] = "busy"

    def set_ready(self):
        '''
        Sets the robot motion to "ready"
            Parameters:
            Returns:
        '''
        self.status["motion"] = "ready"

    def wait_ready(self):
        '''
        Sets the robot motion to "ready"
            Parameters:
            Returns:
        '''
        rate = self.create_rate(2) #2Hz
        while self.status["motion"] != "ready":
            rate.sleep()
            pass

    def move_callback(self, msg):
        '''
        Callback for axidraw_move topic
            Parameters:
            Returns:
        '''
        self.set_busy()

        self.get_logger().info("Received move: {}".format(msg))

        self.ad.goto(msg.x,msg.y)
        self.set_ready()

    def penup_callback(self, msg):
        '''
        Callback for axidraw_penup topic
            Parameters:
            Returns:
        '''
        self.set_busy()

        self.get_logger().info("Received penup: {}".format(msg))

        self.ad.penup()
        self.set_ready()

    def pendown_callback(self, msg):
        '''
        Callback for axidraw_pendown topic
            Parameters:
            Returns:
        '''
        self.set_busy()

        self.get_logger().info("Received pendown: {}".format(msg))

        self.ad.pendown()
        self.set_ready()

    def stroke_callback(self, msg):
        '''
        Callback for axidraw_stroke topic
            Parameters:
            Returns:
        '''
        self.set_busy()

        self.get_logger().info("Received path: {}...".format(msg[:6]))

        path = [ [p.x,p.y] for p in msg.points ]
        self.ad.draw_path(path)
        self.set_ready()


def main(args=None):
    rclpy.init(args=args)

    axidraw_serial = AxidrawSerial()
    try:
        rclpy.spin(axidraw_serial)
    finally:
        axidraw_serial.go_home()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
