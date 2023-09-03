#!/usr/bin/env python3


import tkinter as tk
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math



maxDegree = 90

class SliderPublisher(Node):
    def __init__(self):
        super().__init__('slider_publisher')
        
        self.pub = self.create_publisher(JointState, "joint_states", 1)     #creates a publisher with JointState msg type, "hello_world" topic name, and qos profile size of 10 (queue size of 10)

        self.move_all = False
        
        self.root = tk.Tk()

        self.label1 = tk.Label(self.root, text= "Arm 1" )
        self.label1.grid(row=1, column=1, padx =10, pady = 10)
        self.slider1 = tk.Scale(self.root, from_=maxDegree, to=-maxDegree, length = 200, orient=tk.VERTICAL, command=self.publish_joint_states)
        self.slider1.grid(row=2, column=1, padx =10, pady = 10)

        self.label2 = tk.Label(self.root, text= "Arm 2" )
        self.label2.grid(row=1, column=2, padx =10, pady = 10)
        self.slider2 = tk.Scale(self.root, from_=maxDegree, to=-maxDegree, length = 200, orient=tk.VERTICAL, command=self.publish_joint_states)
        self.slider2.grid(row=2, column=2, padx =10, pady = 10)

        self.label3 = tk.Label(self.root, text= "Arm 3" )
        self.label3.grid(row=1, column=3, padx =10, pady = 10)
        self.slider3 = tk.Scale(self.root, from_=maxDegree, to=-maxDegree, length = 200, orient=tk.VERTICAL, command=self.publish_joint_states)
        self.slider3.grid(row=2, column=3, padx =10, pady = 10)

        self.label4 = tk.Label(self.root, text= "Arm 4" )
        self.label4.grid(row=1, column=4, padx =10, pady = 10)
        self.slider4 = tk.Scale(self.root, from_=maxDegree, to=-maxDegree, length = 200, orient=tk.VERTICAL, command=self.publish_joint_states)
        self.slider4.grid(row=2, column=4, padx =10, pady = 10)

        self.label5 = tk.Label(self.root, text= "Arm 5" )
        self.label5.grid(row=1, column=5, padx =10, pady = 10)
        self.slider5 = tk.Scale(self.root, from_=maxDegree, to=-maxDegree, length = 200, orient=tk.VERTICAL, command=self.publish_joint_states)
        self.slider5.grid(row=2, column=5, padx =10, pady = 10)

        self.label6 = tk.Label(self.root, text= "Arm 6" )
        self.label6.grid(row=1, column=6, padx =10, pady = 10)
        self.slider6 = tk.Scale(self.root, from_=maxDegree, to=-maxDegree, length = 200, orient=tk.VERTICAL, command=self.publish_joint_states)
        self.slider6.grid(row=2, column=6, padx =10, pady = 10)


        self.button = tk.Button(self.root, text= "RESET", command= self.reset_sliders)
        self.button.grid(row=1, column=7, padx = 10, pady = 10)

        self.checkbox = tk.Checkbutton(self.root, text='move all',command=self.toggle_set_all)
        self.checkbox.grid(row=2, column=7, padx = 1, pady=10)


        self.reset_sliders()

    def reset_sliders(self):
        self.slider1.set(0)
        self.slider2.set(0)
        self.slider3.set(0)
        self.slider4.set(0)
        self.slider5.set(0)
        self.slider6.set(0)
    
    def toggle_set_all(self):
        self.move_all = not self.move_all

    def publish_joint_states(self, value):
        if self.move_all:
            self.slider1.set(value)
            self.slider2.set(value)
            self.slider3.set(value)
            self.slider4.set(value)
            self.slider5.set(value)
            self.slider6.set(value)
        

        angle1 = math.radians(self.slider1.get())
        angle2 = math.radians(self.slider2.get())
        angle3 = math.radians(self.slider3.get())
        angle4 = math.radians(self.slider4.get())
        angle5 = math.radians(self.slider5.get())
        angle6 = math.radians(self.slider6.get())

        angles = [angle1, -angle2, angle3, -angle4, angle5, -angle6]

        msg = JointState()    
        msg.header.stamp.sec = rclpy.clock.Clock().now().nanoseconds // 1000000000
        msg.header.stamp.nanosec = rclpy.clock.Clock().now().nanoseconds % 1000000000
        msg.header.frame_id = "" 
        msg.name = ["short_arm1_joint", "short_arm2_joint", "short_arm3_joint", "short_arm4_joint", "short_arm5_joint", "short_arm6_joint", 
                    "long_arm1_joint_x", "long_arm1_joint_z", "long_arm2_joint_x", "long_arm2_joint_z", "long_arm3_joint_x", "long_arm3_joint_z", 
                    "long_arm4_joint_x", "long_arm4_joint_z", "long_arm5_joint_x", "long_arm5_joint_z", "long_arm6_joint_x", "long_arm6_joint_z"]

        
        msg.position = [angles[0], angles[1], angles[2], 
                        angles[3], angles[4], angles[5],
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        msg.velocity = []
        msg.effort = []

        self.pub.publish(msg)


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
