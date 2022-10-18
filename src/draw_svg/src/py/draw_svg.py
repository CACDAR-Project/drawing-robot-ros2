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


class PublishTarget(Node):
    def __init__(self):
        super().__init__('publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/target_pose', 10)
        timer_period = 3.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        #print(p)
        #print(p.position)
        #print(p.orientation)

    def timer_callback(self):
        p = Pose()
        p.position.x = rand(0.1,0.4)
        p.position.y = rand(0.1,0.4)
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
        self.i += 1

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
