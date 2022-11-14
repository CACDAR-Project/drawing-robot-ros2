#!/usr/bin/env python3
"""Example that uses MoveIt 2 to follow a target inside Ignition Gazebo"""

import rclpy
from geometry_msgs.msg import Pose, PoseWithCovariance, Point
from nav_msgs.msg import Odometry
from pymoveit2 import MoveIt2
from robots import lite6
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from queue import Queue

import tkinter as tk
import math

import threading

def translate(val, lmin, lmax, rmin, rmax):
    lspan = lmax - lmin
    rspan = rmax - rmin
    val = float(val - lmin) / float(lspan)
    return rmin + (val * rspan)

class DrawingApp(tk.Tk):
    def __init__(self, queue):
        tk.Tk.__init__(self)

        self.queue = queue
        self.canvas = tk.Canvas(self, width=400, height=400)
        self.canvas.pack(side="top", fill="both", expand=True)
        #self.canvas.create_line(200,200, 200,200, tags=("line",), arrow="last")
        self.read_queue()

    def create_circle(self, x, y, r, **kwargs):
        self.canvas.create_oval(x-r, y-r, x+r, y+r, **kwargs)

    def read_queue(self):
        '''Read queue and draw circle every 100 ms'''
        if self.queue.empty():
            self.after(10, lambda: self.read_queue())
            return
        p = self.queue.get()
        print("last_point->x:{} y:{} z:{}".format(p.x, p.y, p.z))
        r = 5
        x = translate(p.x, -1.0, 0.5, 0, 400)
        y = translate(p.y, 0.5, -1.0, 0, 400)
        self.create_circle(x,y,r, fill="red")
        self.after(10, lambda: self.read_queue())

class LogPen(Node):
    def __init__(self, queue=Queue()):

        self.queue = queue
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
        #self.get_logger().info("x:{} y:{} z:{}".format(p.x, p.y, p.z))
        self.queue.put(p)

def main(args=None):

    rclpy.init(args=args)

    log_pen = LogPen()

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(log_pen)
    executor.spin()

    rclpy.shutdown()
    exit(0)

def log_thread(queue=Queue()):

    rclpy.init()

    log_pen = LogPen(queue)

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(log_pen)
    executor.spin()

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    #main()

    q = Queue()

    thread0 = threading.Thread(target=log_thread, args=[q])
    thread0.start()

    #thread0.join()
    #thread1 = threading.Thread(target=DrawingApp().mainloop)
    #thread1.start()
    #thread1.join()

    app = DrawingApp(q)
    app.mainloop()
