#!/usr/bin/env python3


import tkinter as tk
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
import math
import time
import numpy as np

class FrameCreator():
    def __init__(self, root, name):
    
        self.frame = tk.Frame(root)
        self.button = tk.Button(self.frame, text="Off", command= self.button_function)
        self.on = False
        self.label = tk.Label(self.frame, text= name)
        self.amplitude_entry = tk.Entry(self.frame)
        self.period_entry = tk.Entry(self.frame, )
        self.time_offset_entry = tk.Entry(self.frame)
        self.position_offset_entry = tk.Entry(self.frame)
        self.amplitude_label = tk.Label(self.frame, text= "Amplitude")
        self.period_label = tk.Label(self.frame, text= "Period")
        self.time_offset_label = tk.Label(self.frame, text= "Time Offset")
        self.position_offset_label = tk.Label(self.frame, text= "Position Offset")

        self.button.grid(                    row=1, column=2)
        self.amplitude_entry.grid(          row=2, column=2)
        self.period_entry.grid(             row=3, column=2)        
        self.time_offset_entry.grid(        row=4, column=2)
        self.position_offset_entry.grid(    row=5, column=2)

        self.label.grid(                   row=1, column=1)
        self.amplitude_label.grid(          row=2, column=1)
        self.period_label.grid(             row=3, column=1)
        self.time_offset_label.grid(        row=4, column=1)
        self.position_offset_label.grid(    row=5, column=1)

        self.amplitude_entry.insert(0, "0")
        self.period_entry.insert(0, "8")      
        self.time_offset_entry.insert(0, "0")
        self.position_offset_entry.insert(0, "0")
    
    def button_function(self):
        if self.button.cget("text") == "On":
            self.button.configure(text="Off")
            self.on = False
        else:
            self.button.configure(text="On")
            self.on = True



class SliderPublisher(Node):
    def __init__(self):
        super().__init__('slider_publisher')
        
        self.start_time = time.time()

        self.pub = self.create_publisher(Pose, "end_effector_pose", 1) 
        self.root = tk.Tk()
        self.frames = [FrameCreator(self.root, "X"),
                        FrameCreator(self.root, "Y"),
                        FrameCreator(self.root, "Z"),
                        FrameCreator(self.root, "Pitch"),
                        FrameCreator(self.root, "Roll"),
                        FrameCreator(self.root, "Yaw")]
        
        self.frames[2].position_offset_entry.delete(0,tk.END)
        self.frames[2].position_offset_entry.insert(0,"230")

        for frame in self.frames:
            frame.frame.pack(padx=5,pady=5)

        
        self.oscillate()
        
        
    def oscillate(self):

        values = []

        for frame in self.frames:
            if frame.on:
                try:

                    time_elapsed = time.time() - self.start_time
                    amplitude = float(frame.amplitude_entry.get())
                    period = float(frame.period_entry.get())
                    time_offset = float(frame.time_offset_entry.get())
                    position_offset = float(frame.position_offset_entry.get())
                
                    value = amplitude * math.cos((math.pi*2/period)* (time_elapsed - time_offset)) + position_offset
                except:
                    print("math error")
            else:
                value = 0.0

            values.append(value)
        
        self.publish_values(values)

        self.root.after(20, self.oscillate)


    def publish_values(self, values):

        x = values[0] * 0.001
        y = values[1] * 0.001
        z = values[2] * 0.001

        q = quaternion_from_euler(math.radians(values[3]),
                                  math.radians(values[4]),
                                  math.radians(values[5]))
        

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
