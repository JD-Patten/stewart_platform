#!/usr/bin/env python3

import rclpy                    
from rclpy.node import Node     
from geometry_msgs.msg import Pose, Point, Quaternion
import math
import numpy as np
import os



FRAMERATE = 60
filename = "animation1.txt"

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

#class for creating the node
class AnimationPlayer(Node):        
    def __init__(self):
        super().__init__("animation_player_node")    

        self.read_animation_from_txt()  
        self.frame = 0

        self.pub = self.create_publisher(Pose, "end_effector_pose", 1)     
        self.timer = self.create_timer(1/FRAMERATE, self.publish_animation) 

    def read_animation_from_txt(self):

        # Get the current script's directory
        script_dir = os.path.dirname(os.path.abspath(__file__))



        # Construct the path to the input file relative to the script's directory
        input_file_path = os.path.join(script_dir, "../../share/robot/animation", filename)

        # Open the input file
        input_file = open(input_file_path, "r")

        # Initialize the list of lists
        self.animation_data = []

        # Initialize variables to hold the current frame number and frame data
        frame_num = None
        frame_data = None

        # Loop through each line in the file
        for line in input_file.readlines():
            # If the line starts with "Frame ", it contains the frame number
            if line.startswith("Frame "):
                # If this is not the first frame, add the previous frame data to the animation data list
                if frame_num is not None:
                    self.animation_data.append(frame_data)
                
                # Extract the frame number from the line
                frame_num = int(line.split(" ")[1][0])  # remove the colon at the end
                
                # Create a list to hold the data for this frame
                frame_data = []
            
            # If the line starts with "Location:", it contains location data
            elif line.startswith("Location:"):
                # Extract the location data from the line
                location = [float(s) for s in line.split("<Vector (")[1].split(")>")[0].split(", ")]
                frame_data.extend(location)
            
            # If the line starts with "Rotation:", it contains rotation data
            elif line.startswith("Rotation:"):
                # Extract the rotation data from the line
                rotation = [float(s.split("=")[1]) for s in line.split("<Euler (")[1].split("),")[0].split(", ")]
                frame_data.extend(rotation)

        # Add the last frame data to the animation data list
        if frame_num is not None:
            self.animation_data.append(frame_data)

        # Close the input file
        input_file.close()

        # Print the animation data
        print(self.animation_data)



    def publish_animation(self): 

        x = self.animation_data[self.frame][0]
        y = self.animation_data[self.frame][1]
        z = self.animation_data[self.frame][2] 
        rX = self.animation_data[self.frame][3]
        rY = self.animation_data[self.frame][4]
        rZ = self.animation_data[self.frame][5]

        q = quaternion_from_euler(rX, rY, rZ)

        pose = Pose()     
        pose.position = Point(x= x, y= y, z= z)
        pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.pub.publish(pose)
        self.frame += 1
        if self.frame == 239:
            self.frame = 0       
        


def main(args=None):

    rclpy.init()                        
    my_pub = AnimationPlayer()     
    print("Animation Player Node Running...")

    try:
        rclpy.spin(my_pub)              
    except KeyboardInterrupt:
        print("Terminating Node... ")
        my_pub.destroy_node()


if __name__ == '__main__':
    main()