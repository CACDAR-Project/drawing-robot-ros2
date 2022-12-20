#!/usr/bin/env python3
"""Sends motions individually to robot_controller"""

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile
from random import uniform as rand
import math
#from tf2_ros.transformations import quaternion_from_euler
import lxml.etree as ET

from robot_interfaces.srv import ExecuteMotion
import sys

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = [0,0,0,0]
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

def translate(val, lmin, lmax, rmin, rmax):
    lspan = lmax - lmin
    rspan = rmax - rmin
    val = float(val - lmin) / float(lspan)
    return rmin + (val * rspan)

def map_point_function(x_pixels, y_pixels, xlim_lower, xlim_upper, ylim_lower, ylim_upper):
    def map_point(xpix,ypix):
       x = translate(xpix, 0, x_pixels, xlim_lower, xlim_upper)
       y = translate(ypix, 0, y_pixels, ylim_lower, ylim_upper)
       return (x,y)
    return map_point


class DrawingController(Node):
    def __init__(self, svgpath):
        super().__init__('drawing_controller')
        #self.publisher_ = self.create_publisher(PoseStamped, '/target_pose', 10)
        timer_period = 7.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.cli = self.create_client(ExecuteMotion, 'execute_path')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ExecuteMotion.Request()

        # TODO get dimensions from svg

        #print(p)
        #print(p.position)
        #print(p.orientation)
        xml = ET.parse(svgpath)
        svg = xml.getroot()
        self.map_point = map_point_function(float(svg.get('width')), float(svg.get('height')), 0.1, 0.5, -0.2, 0.2)
        self.points = []
        for child in svg:
            if (child.tag == 'line'):
                self.points.append((float(child.get('x1')), float(child.get('y1'))))
                self.points.append((float(child.get('x2')), float(child.get('y2'))))


    def timer_callback(self):
        next_point = self.points[self.i]
        point = self.map_point(float(next_point[0]),float(next_point[1]))
        p = Pose()
        p.position.x = point[0]
        p.position.y = point[1]
        p.position.z = 0.1
        q = quaternion_from_euler(0.0, math.pi, 0.0)
        #p.orientation = q
        p.orientation.x = q[0]
        p.orientation.y = q[1]
        p.orientation.z = q[2]
        p.orientation.w = q[3]
        ps = PoseStamped()
        ps.pose = p
        #print(ps)
        #self.publisher_.publish(ps)
        #self.get_logger().info('Publishing to /target_pose: "%s"' % p)
        self.i = (self.i + 1) % len(self.points)

        #self.req.a = a
        #self.req.b = b
        self.future = self.cli.call_async(self.req)
        #rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info('Got result: "%s"' % self.future.result())
        #return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    dc = DrawingController(sys.argv[1])

    rclpy.spin(dc)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
