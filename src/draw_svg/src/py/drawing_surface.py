#!/usr/bin/env python3
"""Example that uses MoveIt 2 to follow a target inside Ignition Gazebo"""

import rclpy
from geometry_msgs.msg import Pose, PoseWithCovariance, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image as SensorImage
from pymoveit2 import MoveIt2
from robots import lite6
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from queue import Queue

import tkinter as tk
import math

import threading

from PIL import ImageTk
from PIL import Image as PImage
import numpy as np


def translate(val, lmin, lmax, rmin, rmax):
    lspan = lmax - lmin
    rspan = rmax - rmin
    val = float(val - lmin) / float(lspan)
    return rmin + (val * rspan)

class DrawingApp(tk.Tk):
    def __init__(self, point_queue, image_queue):
        tk.Tk.__init__(self)

        self.point_queue = point_queue
        self.image_queue = image_queue
        self.width = 400
        self.height = 400
        self.img = PImage.new("RGB", (self.width, self.height), (255, 255, 255))
        self.arr = np.array(self.img)
        self.frame_delay = 1 #ms
        self.window_update_delay = 300 #ms
        self.counter = 0
        self.read_queue()

        self.canvas = tk.Canvas(self,width=self.width,height=self.height)


        self.tk_image = ImageTk.PhotoImage(self.img)
        self.canvas.create_image(self.width/2,self.height/2,image=self.tk_image)
        self.canvas.pack(side='top', expand=True, fill='both')

        def reset():
            self.img = PImage.new("RGB", (self.width, self.height), (255, 255, 255))
            self.arr = np.array(self.img)
        tk.Button(self.canvas, text= "reset", command=reset).pack()

        self.draw_window()

    def draw(self,x,y,z):
        # putpixel is too slow
        #self.img.putpixel((int(x), int(y)), (255, 0, 0))
        self.arr[x,y,1] = 0
        self.arr[x,y,2] = 0
        self.arr[x+1,y,1] = 0
        self.arr[x+1,y,2] = 0
        self.arr[x+1,y+1,1] = 0
        self.arr[x+1,y+1,2] = 0
        self.arr[x,y+1,1] = 0
        self.arr[x,y+1,2] = 0
        #print("Set x:{} y:{} to:{}".format(x,y,255))

    def draw_window(self):
        self.img = PImage.fromarray(self.arr)
        self.tk_image = ImageTk.PhotoImage(self.img)
        self.canvas.create_image(self.width/2,self.height/2,image=self.tk_image)
        self.image_queue.put(self.get_image_message())
        #self.after(self.window_update_delay, lambda: self.draw_window())

    def read_queue(self):
        '''Read queue and draw circle every 10 ms'''
        self.counter = self.counter + self.frame_delay
        if self.point_queue.empty():
            self.after(self.frame_delay, lambda: self.read_queue())
            return

        while not self.point_queue.empty():
            p = self.point_queue.get()
            #x = translate(p.x, -1.0, 0.5, 0, self.width)
            #y = translate(p.y, 0.5, -1.0, 0, self.height)
            x = translate(p.x, -1.0, 0.5, 0, self.width)
            y = translate(p.y, -1.0, 0.5, 0, self.height)
            self.draw(int(x),int(y),0)

        if self.counter >= self.window_update_delay:
            #print("Redraw")
            self.counter = 0
            self.draw_window()

        self.after(self.frame_delay, lambda: self.read_queue())

    # https://stackoverflow.com/questions/64373334/how-can-i-publish-pil-image-binary-through-ros-without-opencv
    def get_image_message(self):
        msg = SensorImage()
        #msg.header.stamp = rospy.Time.now()
        msg.height = self.img.height
        msg.width = self.img.width
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = 3 * self.img.width
        msg.data = np.array(self.img).tobytes()
        return msg

class LogPen(Node):
    def __init__(self, point_queue=Queue()):

        self.queue = point_queue
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

class PublishImage(Node):
    def __init__(self, image_queue=Queue()):

        self.image_queue = image_queue
        super().__init__("publish_image")

        self.publisher_ = self.create_publisher(SensorImage, '/drawing_surface/image_raw', 10)
        timer_period = 0.0002  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Initialization successful.")

    def timer_callback(self):
        """
        Output image from queue
        """
        if self.image_queue.empty():
            return

        img = self.image_queue.get()
        #self.get_logger().info("Publishing image {}x{}".format(img.width,img.height))

        self.publisher_.publish(img)



def main(args=None):

    rclpy.init(args=args)

    log_pen = LogPen()

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(log_pen)
    executor.spin()

    rclpy.shutdown()
    exit(0)

def start_node_thread(constructor, queue=Queue()):

    node = constructor(queue)

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor.spin()


if __name__ == "__main__":
    point_queue = Queue()
    image_queue = Queue()

    rclpy.init()

    global app
    app = DrawingApp(point_queue, image_queue)

    global log_thread
    log_thread = threading.Thread(target=start_node_thread, args=[LogPen, point_queue])
    log_thread.start()

    global image_thread
    image_thread = threading.Thread(target=start_node_thread, args=[PublishImage, image_queue])
    image_thread.start()

    app.mainloop()
    rclpy.shutdown()
    exit(0)
