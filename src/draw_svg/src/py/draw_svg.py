#!/usr/bin/env python3
"""Example that uses MoveIt 2 to follow a target inside Ignition Gazebo"""

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from pymoveit2 import MoveIt2
from pymoveit2.robots import panda
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile
from random import uniform as rand
import math
#from tf2_ros.transformations import quaternion_from_euler
import lxml.etree as ET


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


class PublishTarget(Node):
    def __init__(self):
        super().__init__('publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/target_pose', 10)
        timer_period = 3.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        # TODO get dimensions from svg

        #print(p)
        #print(p.position)
        #print(p.orientation)
        xml = ET.parse('svg/test.svg')
        svg = xml.getroot()
        self.map_point = map_point_function(float(svg.get('width')), float(svg.get('height')), 0.0, 0.5, 0.3, 0.8)
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
        self.publisher_.publish(ps)
        self.get_logger().info('Publishing to /target_pose: "%s"' % p)
        self.i = (self.i + 1) % len(self.points)

def main(args=None):
    rclpy.init(args=args)

    publisher = PublishTarget()

    rclpy.spin(publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
