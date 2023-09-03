#!/usr/bin/env python3

import rclpy                    
from rclpy.node import Node     
from geometry_msgs.msg import Pose, Point, Quaternion
import math

#class for creating the node
class ConstantPosePublisher(Node):        
    def __init__(self):
        super().__init__("constant_pose_publisher")                        
        self.pub = self.create_publisher(Pose, "end_effector_pose", 1)     
        self.timer = self.create_timer(0.03, self.publish_hello_world) 
        self.counter = 0


    def publish_hello_world(self): 
        zValue = .132
        pose = Pose()     
        pose.position = Point(x=0.0, y=0.0, z= zValue)
        pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.pub.publish(pose)        
        self.counter += 1
        


def main(args=None):
    rclpy.init()                        
    my_pub = ConstantPosePublisher()     
    print("Constant Pose Publisher Node Running...")

    try:
        rclpy.spin(my_pub)              
    except KeyboardInterrupt:
        print("Terminating Node... ")
        my_pub.destroy_node()


if __name__ == '__main__':
    main()