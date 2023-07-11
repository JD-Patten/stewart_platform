#!/usr/bin/env python3


import tkinter as tk
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
import math


class SliderPublisher(Node):
    def __init__(self):
        super().__init__('slider_publisher')
        self.pub = self.create_publisher(Pose, "end_effector_pose", 1)     

        self.root = tk.Tk()
        self.slider = tk.Scale(self.root, from_=100, to=200, orient=tk.HORIZONTAL, command=self.publish_slider_value)
        self.slider.pack()

    

    def publish_slider_value(self, value):
        msg = Pose()
        msg.position = Point(x=0.0, y=0.0, z= int(value) * 0.001)
        msg.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.pub.publish(msg)
        self.get_logger().info(f'Published slider value: {msg.position}')

    def run(self):
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    slider_publisher = SliderPublisher()
    slider_publisher.run()
    slider_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
