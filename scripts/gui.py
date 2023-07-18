#!/usr/bin/env python3


import tkinter as tk
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
import math
import numpy as np


class SliderPublisher(Node):
    def __init__(self):
        super().__init__('slider_publisher')
        self.pub = self.create_publisher(Pose, "end_effector_pose", 1) 

        self.root = tk.Tk()

        self.xLabel = tk.Label(self.root, text= "X (mm)" )
        self.xLabel.grid(row=1, column=1, padx =10, pady = 10)
        self.xSlider = tk.Scale(self.root, from_=-100, to=100, length = 400, orient=tk.HORIZONTAL, command=self.publish_slider_value)
        self.xSlider.grid(row=2, column=1, padx =10, pady = 10)

        self.yLabel = tk.Label(self.root, text= "Y (mm)" )
        self.yLabel.grid(row=3, column=1, padx =10, pady = 10)
        self.ySlider = tk.Scale(self.root, from_=-100, to=100, length = 400, orient=tk.HORIZONTAL, command=self.publish_slider_value)
        self.ySlider.grid(row=4, column=1, padx =10, pady = 10)
        
        self.zLabel = tk.Label(self.root, text= "Z (mm)" )
        self.zLabel.grid(row=5, column=1, padx =10, pady = 10)
        self.zSlider = tk.Scale(self.root, from_=170, to=370, length = 400, orient=tk.HORIZONTAL, command=self.publish_slider_value)
        self.zSlider.grid(row=6, column=1, padx =10, pady = 10)

        self.aLabel = tk.Label(self.root, text= "Pitch (deg)" )
        self.aLabel.grid(row=1, column=2, padx =10, pady = 10)
        self.aSlider = tk.Scale(self.root, from_= -90, to=90, length = 400, orient=tk.HORIZONTAL, command=self.publish_slider_value)
        self.aSlider.grid(row=2, column=2, padx =10, pady = 10)

        self.bLabel = tk.Label(self.root, text= "Roll (deg)" )
        self.bLabel.grid(row=3, column=2, padx =10, pady = 5)
        self.bSlider = tk.Scale(self.root, from_= -90, to=90, length = 400, orient=tk.HORIZONTAL, command=self.publish_slider_value)
        self.bSlider.grid(row=4, column=2, padx =10, pady = 5)
        
        self.cLabel = tk.Label(self.root, text= "Yaw (deg)" )
        self.cLabel.grid(row=5, column=2, padx =10, pady = 10)
        self.cSlider = tk.Scale(self.root, from_= -180, to=180, length = 400, orient=tk.HORIZONTAL, command=self.publish_slider_value)
        self.cSlider.grid(row=6, column=2, padx =10, pady = 10)

        self.button = tk.Button(self.root, text= "RESET", command= self.reset_sliders)
        self.button.grid(row=1, column=3, padx = 10, pady = 10)

        self.reset_sliders()
        

    def reset_sliders(self):
        self.xSlider.set(0)
        self.ySlider.set(0)
        self.zSlider.set(270)
        self.aSlider.set(0)
        self.bSlider.set(0)
        self.cSlider.set(0)

    def publish_slider_value(self, value):

        x = self.xSlider.get() * 0.001
        y = self.ySlider.get() * 0.001
        z = self.zSlider.get() * 0.001

        q = quaternion_from_euler(math.radians(self.aSlider.get()),
                                  math.radians(self.bSlider.get()),
                                  math.radians(self.cSlider.get()))
        
        print(x)
        print(y)
        msg = Pose()
        msg.position = Point(x=x, y=y, z=z)
        msg.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.pub.publish(msg)
        self.get_logger().info(f'Published slider value: {msg.position}')

    def run(self):
        self.root.mainloop()

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

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

def main(args=None):
    rclpy.init(args=args)
    slider_publisher = SliderPublisher()
    slider_publisher.run()
    slider_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
