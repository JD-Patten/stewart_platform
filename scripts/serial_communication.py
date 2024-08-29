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
        

        self.sub = self.create_subscription(JointState,'joint_states', self.subscriber_callback, 1) 

        #set the offsets to account for error on the actual platform
        #a positive number causes a counter-clockwise offset and is measured in degrees
        self.offsets = [-5, 0, 2, 0, 4, 1]


        #set the initial angles
        self.angles = [90, 90, 90, 90, 90, 90]

        #names of the joints that have the servo angles in the JointState message
        self.joint_names = ['short_arm1_joint',
                            'short_arm2_joint',
                            'short_arm3_joint',
                            'short_arm4_joint',
                            'short_arm5_joint',
                            'short_arm6_joint']


        #set up the connection on the right port for serial communication
        port = '/dev/ttyUSB0'

        self.get_logger().info(f'Setting up serial communication on port: {port}')
        self.ser = serial.Serial(port, 57600)
        time.sleep(5)
        self.get_logger().info(f'Serial communication ready')



    def subscriber_callback(self, msg): 
        joint_names = msg.name

        #find where the angles are in the JointState message and save them in the correct order in the angles list
        try:
            servo_indeces = [joint_names.index(name) for name in self.joint_names]


        #if all the angles can't be found in the message dont move on with publishing a message
        except ValueError:
            return
        
        for i, index in enumerate(servo_indeces):
            self.angles[i] = math.degrees(msg.position[index])

        

        #Publish the angles
        self.broadcast()

    def broadcast(self):
        
        #the servos are centered at 90 degrees, so to convert angles to servo positions 90 degrees needs to be added to them
        servo_positions = [90 + angle + offset for angle, offset in zip(self.angles, self.offsets)]

        #The command to be broadcasted is a string containing the anges seperated by indeces labeled a,b,c,d,e
        # Example: 1.0a3.2b13.892c2.3d33.33e8.876\n
        command = f'{servo_positions[0]}a{servo_positions[1]}b{servo_positions[2]}c{servo_positions[3]}d{servo_positions[4]}e{servo_positions[5]}\n'



        self.ser.write(command.encode())

        
            

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

