#!/usr/bin/env python3
"""Example that uses MoveIt 2 to follow a target inside Ignition Gazebo"""

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from pymoveit2 import MoveIt2, MoveIt2Servo
from robots import lite6
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile
import math
import random


class FollowTarget(Node):
    def __init__(self):

        super().__init__("follow_py")

        # Create callback group that allows execution of callbacks in parallel without restrictions
        self._callback_group = ReentrantCallbackGroup()

        ## Create MoveIt 2 interface
        #self._moveit2 = MoveIt2(
        #    node=self,
        #    joint_names=lite6.joint_names(),
        #    base_link_name=lite6.base_link_name(),
        #    end_effector_name=lite6.end_effector_name(),
        #    group_name=lite6.MOVE_GROUP_ARM,
        #    execute_via_moveit=True,
        #    callback_group=self._callback_group,
        #)
        ## Use upper joint velocity and acceleration limits
        #self._moveit2.max_velocity = 1.0
        #self._moveit2.max_acceleration = 1.0


        # Create a subscriber for target pose
        p = Pose()
        p.position.x=0.0
        p.position.y=0.0
        p.position.z=0.5
        p.orientation.x=0.0
        p.orientation.y=1.0
        p.orientation.z=0.0
        p.orientation.w=0.0
        self.__previous_target_pose = p

        self.get_logger().info("Connect to moveit2 servo node")
        # https://github.com/AndrejOrsula/pymoveit2/blob/master/examples/ex_servo.py
        # Create MoveIt 2 Servo interface
        self._moveit2_servo = MoveIt2Servo(
            node=self,
            frame_id=lite6.base_link_name(),
            callback_group=self._callback_group,
            enable_at_init=False,
            linear_speed=1.0,
            angular_speed=1.0,
        )
        self._moveit2_servo.enable(sync=False)

        self.get_logger().info("Moving to initial position")


        #self._moveit2.move_to_pose(
        #    position=p.position,
        #    quat_xyzw=p.orientation,
        #    cartesian=True,
        #)
        self._moveit2_servo.servo(linear=(p.position.x, p.position.y, p.position.z), angular=(0.0, 0.0, 0.0))

        self.create_subscription(
            msg_type=PoseStamped,
            topic="/target_pose",
            callback=self.target_pose_callback,
            qos_profile=QoSProfile(depth=1),
            callback_group=self._callback_group,
        )

        self.get_logger().info("Initialization successful.")

    def target_pose_callback(self, msg: PoseStamped):
        """
        Plan and execute trajectory each time the target pose is changed
        """

        p = msg.pose
        # Return if target pose is unchanged
        if p == self.__previous_target_pose:
            return

        self.get_logger().info("Target pose has changed. Planning and executing...")

        # Options https://github.com/AndrejOrsula/pymoveit2/blob/master/pymoveit2/moveit2.py
        # Plan and execute motion
        #self._moveit2.move_to_pose(
        #    position=msg.pose.position,
        #    quat_xyzw=msg.pose.orientation,
        #    cartesian=True,
        #)
        pos = p.position
        ori = p.orientation

        def r():
            return random.uniform(-1.0,1.0)

        pos.x = r()
        pos.y = r()
        pos.z = 0.0
        ori.x = r()
        ori.y = r()
        ori.z = r()
        self.get_logger().info("pos:{} \nori:{}".format(pos,ori))
        #self._moveit2_servo.servo(linear=(pos.x, pos.y, pos.z), angular=(0.0, 0.0, 0.0))
        self._moveit2_servo.servo(linear=(pos.x, pos.y, pos.z), angular=(ori.x, ori.y, ori.z))

        #now_sec = self.get_clock().now().nanoseconds * 1e-9
        #self._moveit2_servo(linear=(math.sin(now_sec), math.cos(now_sec), 0.0), angular=(0.0, 0.0, 0.0))


        # Update for next callback
        self.__previous_target_pose = msg.pose

def main(args=None):

    rclpy.init(args=args)

    target_follower = FollowTarget()

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(target_follower)
    executor.spin()

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()
