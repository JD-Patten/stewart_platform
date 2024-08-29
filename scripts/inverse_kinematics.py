#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from sensor_msgs.msg import JointState


#lengths of the parts of the arms

l1 = 65 * 0.001 # mm to m 
l2 = 194 * 0.001 # mm to m


class FrameListener(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_node_with_long_arms')

        
        self.pub = self.create_publisher(JointState, "joint_states", 1)    

        self.servo_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Call on_timer function every second
        self.timer = self.create_timer(0.03, self.on_timer)


    def on_timer(self):

        from_frames = ['arm1_end_connection', 'arm2_end_connection', 'arm3_end_connection', 
                       'arm4_end_connection', 'arm5_end_connection', 'arm6_end_connection']
        to_frames = ['servo1_link', 'servo2_link', 'servo3_link', 
                     'servo4_link', 'servo5_link', 'servo6_link']
        
        servo_angles = []
        axs = []
        ays = []

        try:

            for i in range(6):
                try:
                    t = self.tf_buffer.lookup_transform(to_frames[i], from_frames[i], rclpy.time.Time())

                except TransformException as ex:
                    self.get_logger().info(f'Could not transform {to_frames[i]} to {from_frames[i]}: {ex}')
                    print("Could not transform" +str(i+1))
                    return

                x = t.transform.translation.x
                y = t.transform.translation.y
                z = t.transform.translation.z 


                servo_angle, ax, ay = self.solve_for_angle(x, y, z)
                servo_angles.append(servo_angle)
                axs.append(ax)
                ays.append(ay)

            self.publish_joint_states(servo_angles, axs, ays)

        except:
            print("*** error calculating angles ***")



    def solve_for_angle(self, x, y, z):     

        #Inverse Kinematics Equations

        #adding and subtracting the acos portion decides which of the two possible solutions to choose
        #using if z < 0 uses different solutions for the upside down servos. this keeps the arms pointing outwards
        
        if z < 0:
            angle = math.atan2(z,y) + math.acos((l2**2 - y**2 - z**2 - x**2 - l1**2) / (-2 * l1 * math.sqrt(y**2 + z**2)))

            anglex = math.atan2(z - (l1 * math.sin(angle)), y - (l1 * math.cos(angle))) - angle + math.pi

            angley = -1 * math.atan2(math.sqrt((y - (l1 * math.cos(angle)))**2 + (z - (l1 * math.sin(angle)))**2),x) - (math.pi/2)

        else:
            angle = math.atan2(z,y) - math.acos((l2**2 - y**2 - z**2 - x**2 - l1**2) / (-2 * l1 * math.sqrt(y**2 + z**2)))

            anglex = math.atan2(z - (l1 * math.sin(angle)), y - (l1 * math.cos(angle))) - angle
            
            angley = math.atan2(math.sqrt((y - (l1 * math.cos(angle)))**2 + (z - (l1 * math.sin(angle)))**2),x) - (math.pi/2)

        return angle, anglex, angley
    
     
    def publish_joint_states(self, servo_angles, axs, ays):      
        msg = JointState()    
        msg.header.stamp.sec = rclpy.clock.Clock().now().nanoseconds // 1000000000
        msg.header.stamp.nanosec = rclpy.clock.Clock().now().nanoseconds % 1000000000
        msg.header.frame_id = "" 
        msg.name = ["short_arm1_joint", "short_arm2_joint", "short_arm3_joint",
                    "short_arm4_joint", "short_arm5_joint", "short_arm6_joint", 
                    "elbow1_x_joint", "elbow2_x_joint", "elbow3_x_joint", "elbow4_x_joint", "elbow5_x_joint", "elbow6_x_joint",
                    "elbow1_y_joint", "elbow2_y_joint", "elbow3_y_joint", "elbow4_y_joint", "elbow5_y_joint", "elbow6_y_joint"]

        
        msg.position = [servo_angles[0], servo_angles[1], servo_angles[2], 
                        servo_angles[3], servo_angles[4], servo_angles[5],
                        axs[0], axs[1], axs[2], axs[3], axs[4], axs[5],
                       ays[0], ays[1], ays[2], ays[3], ays[4], ays[5]]


        msg.velocity = []
        msg.effort = []

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
