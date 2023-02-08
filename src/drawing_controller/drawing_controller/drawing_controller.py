#!/usr/bin/env python3
"""Sends motions individually to robot_controller"""

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile
from random import uniform as rand
import math
#from tf2_ros.transformations import quaternion_from_euler
import lxml.etree as ET

from rclpy.action import ActionClient
from robot_interfaces.action import ExecuteMotion
from robot_interfaces.msg import Motion
import sys
from copy import deepcopy

from drawing_controller.svg_processor import SVGProcessor

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
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.busy = False

        self._action_client = ActionClient(self, ExecuteMotion, 'execute_motion')

        xml = ET.parse(svgpath)
        svg = xml.getroot()
        #self.map_point = map_point_function(float(svg.get('width')), float(svg.get('height')), 0.1, 0.5, -0.2, 0.2)
        try:
            self.map_point = map_point_function(float(svg.get('width')), float(svg.get('height')), 0.0, 1.0, 0.0, 1.0)
        except:
            self.map_point = map_point_function(1000, 1000, 0.0, 1.0, 0.0, 1.0)

        self.lines = [ ((0,0),(0,0)) ]
        for child in svg:
            if (child.tag == 'line'):
                p1 = (float(child.get('x1')), float(child.get('y1')))
                p2 = (float(child.get('x2')), float(child.get('y2')))
                self.lines.append((p1,p2))

        self.svg_processor = SVGProcessor(self.get_logger())
        self.svg = self.svg_processor.process_svg(svgpath)
        self.get_logger().info('Ready to begin executing motions')

    def send_goal(self, motion):
        self.busy = True
        goal_msg = ExecuteMotion.Goal()
        goal_msg.motion = motion

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        self.busy = False

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback))

    def append_points(self, motion, points):
        for point in points:
            p = Pose()
            #self.get_logger().info('Appending point:{} {}'.format(point[0], point[1]))
            p.position.x = float(point[0])
            p.position.y = float(point[1])
            p.position.z = float(point[2])
            q = quaternion_from_euler(0.0, math.pi, 0.0)
            p.orientation = Quaternion()
            p.orientation.x = q[0]
            p.orientation.y = q[1]
            p.orientation.z = q[2]
            p.orientation.w = q[3]
            ps = PoseStamped()
            ps.pose = p
            motion.path.append(ps)

    def timer_callback(self):
        if self.busy:
            return
        if self.i >= len(self.svg):
            self.get_logger().info('Finished executing all motions from SVG')
            exit()
        next_motion = self.svg[self.i]
        motion = Motion()
        self.append_points(motion, next_motion)
        self.i = self.i + 1
        self.get_logger().info('Executing motion: {}...'.format(motion.path[:10]))
        self.send_goal(motion)


def main(args=None):
    rclpy.init(args=args)

    dc = DrawingController(sys.argv[1])

    rclpy.spin(dc)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
