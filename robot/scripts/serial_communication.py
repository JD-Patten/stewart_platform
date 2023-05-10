#!/usr/bin/env python3



import rclpy 
from rclpy.node import Node     
from sensor_msgs.msg import JointState

import serial
import time
import math


class SerialComNode(Node):
    def __init__(self):
        super().__init__("serial_communication_node", ) 
        
        self.sub = self.create_subscription(JointState,'joint_states', self.subscriber_callback, 1) #creates the subscription that recieves String messages from std_msgs, listens on the topic "hello_world", runs the self.subscriber_callback function after each message, and history depth of 10

        self.angle1 = 90
        self.angle2 = 90
        self.angle3 = 90
        self.angle4 = 90
        self.angle5 = 90
        self.angle6 = 90

        print("Setting Up Serial Communication...")
        self.ser = serial.Serial('/dev/ttyUSB0', 57600)
        time.sleep(5)
        print("Serial Communication Ready")

        self.counter = 0
        #self.timer = self.create_timer(0.15, self.broadcast)


    def subscriber_callback(self, msg): 
        joint_names = msg.name

        try:
            servo1_index = joint_names.index('short_arm1_joint')
            servo2_index = joint_names.index('short_arm2_joint')
            servo3_index = joint_names.index('short_arm3_joint')
            servo4_index = joint_names.index('short_arm4_joint')
            servo5_index = joint_names.index('short_arm5_joint')
            servo6_index = joint_names.index('short_arm6_joint')



        except ValueError:
            return
        
        self.angle1 = math.degrees(msg.position[servo1_index])
        self.angle2 = math.degrees(msg.position[servo2_index])
        self.angle3 = math.degrees(msg.position[servo3_index])
        self.angle4 = math.degrees(msg.position[servo4_index])
        self.angle5 = math.degrees(msg.position[servo5_index])
        self.angle6 = math.degrees(msg.position[servo6_index])

        self.broadcast()

    def broadcast(self):

        pos1 = 90 + self.angle1
        pos2 = 90 + self.angle2
        pos3 = 90 + self.angle3
        pos4 = 90 + self.angle4
        pos5 = 90 + self.angle5
        pos6 = 90 + self.angle6

        #print(pos1 + 'a' + pos2 + 'b' + pos3 + 'c' + pos4 + 'd' + pos5 + 'e' + pos6 + '\n')
        self.ser.write(str.encode(str(pos1) + 'a' + str(pos2) + 'b' + str(pos3) + 'c' + str(pos4) + 'd' + str(pos5) + 'e' + str(pos6) + '\n'))
        
        #time.sleep(0.05)

            

def main(args=None):
    rclpy.init()           
    my_sub = SerialComNode()      

    try:
        rclpy.spin(my_sub)              
    except KeyboardInterrupt:
        print("Terminating Node... ")
        my_sub.destroy_node()


if __name__ == '__main__':
    main()

