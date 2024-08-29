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
        self.button = tk.Button(self.frame, text="Off", command=self.button_function)
        self.on = False
        self.label = tk.Label(self.frame, text=name)
        self.amplitude_entry = tk.Entry(self.frame)
        self.period_entry = tk.Entry(self.frame)
        self.time_shift_entry = tk.Entry(self.frame)
        self.position_shift_entry = tk.Entry(self.frame)
        self.amplitude_label = tk.Label(self.frame, text="Amplitude")
        self.period_label = tk.Label(self.frame, text="Period")
        self.time_shift_label = tk.Label(self.frame, text="Time Offset")
        self.position_shift_label = tk.Label(self.frame, text="Position Offset")

        self.button.grid(row=1, column=2)
        self.amplitude_entry.grid(row=2, column=2)
        self.period_entry.grid(row=3, column=2)
        self.time_shift_entry.grid(row=4, column=2)
        self.position_shift_entry.grid(row=5, column=2)

        self.label.grid(row=1, column=1)
        self.amplitude_label.grid(row=2, column=1)
        self.period_label.grid(row=3, column=1)
        self.time_shift_label.grid(row=4, column=1)
        self.position_shift_label.grid(row=5, column=1)

        self.amplitude_entry.insert(0, "0")
        self.period_entry.insert(0, "8")
        self.time_shift_entry.insert(0, "0")
        self.position_shift_entry.insert(0, "0")
    
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

        # Create frames for each degree of freedom (DOF)
        self.frames = [FrameCreator(self.root, "X"),
                       FrameCreator(self.root, "Y"),
                       FrameCreator(self.root, "Z"),
                       FrameCreator(self.root, "Roll"),
                       FrameCreator(self.root, "Pitch"),
                       FrameCreator(self.root, "Yaw")]
        
        self.frames[2].position_shift_entry.delete(0, tk.END)
        self.frames[2].position_shift_entry.insert(0, "230")

        # Pack frames in a single column
        for frame in self.frames:
            frame.frame.grid(row=self.frames.index(frame), column=0, padx=5, pady=5)

        # Add the additional text field to the right and the buttons
        self.additional_frame = tk.Frame(self.root)
        self.copy_button = tk.Button(self.additional_frame, text="Copy from dictionary", command=self.copy_from_dictionary)
        self.additional_text = tk.Text(self.additional_frame, width=40, height=10)
        self.period_label = tk.Label(self.additional_frame, text="New Period")
        self.period_entry = tk.Entry(self.additional_frame)
        self.change_periods_button = tk.Button(self.additional_frame, text="Change Periods", command=self.change_periods)

        self.copy_button.grid(row=0, column=0, padx=5, pady=5)
        self.additional_text.grid(row=1, column=0, padx=5, pady=5)
        self.period_label.grid(row=2, column=0, padx=5, pady=5)
        self.period_entry.grid(row=3, column=0, padx=5, pady=5)
        self.change_periods_button.grid(row=4, column=0, padx=5, pady=5)

        self.additional_frame.grid(row=0, column=1, rowspan=len(self.frames), padx=5, pady=5)

        self.oscillate()
    
    def copy_from_dictionary(self):
        try:
            # Get the dictionary from the text widget
            dictionary_text = self.additional_text.get("1.0", tk.END)
            dictionary = eval(dictionary_text)
            
            # Map frame names to the dictionary keys
            frame_map = {'X': 'x', 'Y': 'y', 'Z': 'z', 'Roll': 'roll', 'Pitch': 'pitch', 'Yaw': 'yaw'}
            
            for frame in self.frames:
                key = frame_map[frame.label.cget("text")]
                data = dictionary[key]
                
                frame.amplitude_entry.delete(0, tk.END)
                frame.amplitude_entry.insert(0, str(data['amplitude']))
                
                frame.period_entry.delete(0, tk.END)
                frame.period_entry.insert(0, str(data['period']))
                
                frame.position_shift_entry.delete(0, tk.END)
                frame.position_shift_entry.insert(0, str(data['position shift']))
                
                frame.time_shift_entry.delete(0, tk.END)
                frame.time_shift_entry.insert(0, str(data['time shift']))
        except Exception as e:
            print(f"Error copying from dictionary: {e}")

    def change_periods(self):
        try:
            new_period = float(self.period_entry.get())
            for frame in self.frames:
                frame.period_entry.delete(0, tk.END)
                frame.period_entry.insert(0, str(new_period))
        except Exception as e:
            print(f"Error changing periods: {e}")

    def oscillate(self):
        values = []

        for frame in self.frames:
            if frame.on:
                try:
                    time_elapsed = time.time() - self.start_time
                    amplitude = float(frame.amplitude_entry.get())
                    period = float(frame.period_entry.get())
                    time_shift = float(frame.time_shift_entry.get())
                    position_shift = float(frame.position_shift_entry.get())
                
                    value = amplitude * math.cos((math.pi * 2 / period) * (time_elapsed - (time_shift * period))) + position_shift
                except:
                    print("math error")
            else:
                value = 0.0

            values.append(value)
        
        self.publish_values(values)

        self.root.after(20, self.oscillate)

    def publish_values(self, values):
        x = values[0] * 0.001 * -1/2
        y = values[1] * 0.001 *  1/2
        z = values[2] * 0.001

        q = quaternion_from_euler(math.radians(values[3]) * -1,
                                  math.radians(values[4]) * -1,
                                  math.radians(values[5]) * -1)

        msg = Pose()
        msg.position = Point(x=x, y=y, z=z)
        msg.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.pub.publish(msg)
        self.get_logger().debug(f'Published slider value: {msg.position}')

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
    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk

    q = np.empty((4,))
    q[0] = cj * sc - sj * cs
    q[1] = cj * ss + sj * cc
    q[2] = cj * cs - sj * sc
    q[3] = cj * cc + sj * ss

    return q

def main(args=None):
    rclpy.init(args=args)
    slider_publisher = SliderPublisher()
    slider_publisher.run()
    slider_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
