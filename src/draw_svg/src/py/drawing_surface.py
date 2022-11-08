#!/usr/bin/env python3
"""Example that uses MoveIt 2 to follow a target inside Ignition Gazebo"""

import rclpy
from geometry_msgs.msg import Pose, PoseWithCovariance
from nav_msgs.msg import Odometry
from pymoveit2 import MoveIt2
from robots import lite6
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class LogPen(Node):
    def __init__(self):

        super().__init__("log_pen")

        # Create callback group that allows execution of callbacks in parallel without restrictions
        self._callback_group = ReentrantCallbackGroup()

        self.create_subscription(
            msg_type=Odometry,
            topic="/pen_position",
            callback=self.pen_position_callback,
            qos_profile=QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                   history=HistoryPolicy.KEEP_LAST,
                                   depth=1),
            callback_group=self._callback_group,
        )
        self.get_logger().info("Initialization successful.")

    def pen_position_callback(self, msg: Odometry):
        """
        Log position of pen
        """
        p = msg.pose.pose.position
        #self.get_logger().info("x:{}, y:{}, z:{}".format(p[0], p[1], p[2]))
        self.get_logger().info("x:{} y:{} z:{}".format(p.x, p.y, p.z))

def main(args=None):

    rclpy.init(args=args)

    log_pen = LogPen()

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(log_pen)
    executor.spin()

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()
