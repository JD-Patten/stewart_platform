#!/usr/bin/env python3

import math
import sympy
from sympy import Eq, Symbol as sym, solve

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint



l1 = 77 * 0.001 # mm to m
l2 = 235 *0.001 # mm to m

# servo1_offset = math.radians(0.0)
# servo2_offset = math.radians(-15.0)
# servo3_offset = math.radians(0.0)
# servo4_offset = math.radians(-1.0)
# servo5_offset = math.radians(-1.0)
# servo6_offset = math.radians(-1.0)


class FrameListener(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_node')

        # Declare and acquire `target_frame` parameter
        self.pub = self.create_publisher(JointTrajectory, "set_joint_trajectory", 1)     #creates a publisher with JointTrajectory msg type, "hello_world" topic name, and qos profile size of 10 (queue size of 10)

        self.servo_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        #self.solveInverseKinematicsEquation()

        # Call on_timer function every second
        self.timer = self.create_timer(0.03, self.on_timer)


    def on_timer(self):

        from_frames = ['arm1_end_connection', 'arm2_end_connection', 'arm3_end_connection', 
                       'arm4_end_connection', 'arm5_end_connection', 'arm6_end_connection']
        to_frames = ['servo1_link', 'servo2_link', 'servo3_link', 
                     'servo4_link', 'servo5_link', 'servo6_link']
        angles = [0, 0, 0, 0, 0, 0]

        try:

            for i in range(6):
                try:
                    t = self.tf_buffer.lookup_transform(to_frames[i], from_frames[i], rclpy.time.Time())
                except TransformException as ex:

                    self.get_logger().info(
                        f'Could not transform {to_frames[i]} to {from_frames[i]}: {ex}')
                    print("Could not transform" +str(i+1))
                    return

                x = t.transform.translation.x
                y = t.transform.translation.y
                z = t.transform.translation.z 


                angle = self.solve_for_angle(x, y, z)
                angles[i] = angle

            self.publish_set_joint_trajectory(angles)

        except:
            print("*** error calculating angles ***")

    def solveInverseKinematicsEquation(self):

        print("solving...")

        l1 = sym('l1')
        l2 = sym('l2')
        x2 = sym('x2')
        y2 = sym('y2')
        z2 = sym('z2')
        theta = sym('theta')

        equation = Eq(l2**2, (x2 - (l1 * sympy.cos(theta)) ) **2 + (y2-(l1 * sympy.sin(theta)))**2 + z2**2)
        self.solution = solve(equation, theta, minimal=True)

        return

    def solve_for_angle(self, x, y, z):     

        #Inverse Kinematics Equations
        #adding and subtracting the acos portion decides which of the two possible solutions to choose
        #using if z < 0 uses different solutions for the upside down servos. this keeps the arms pointing outwards
        
        if z < 0:
            angle = math.atan2(z,y) + math.acos((l2**2 - y**2 - z**2 - x**2 - l1**2) / (-2 * l1 * math.sqrt(y**2 + z**2)))
        else:
            angle = math.atan2(z,y) - math.acos((l2**2 - y**2 - z**2 - x**2 - l1**2) / (-2 * l1 * math.sqrt(y**2 + z**2)))

        return angle
    
    def publish_set_joint_trajectory(self, angles):      
        msg = JointTrajectory()    
        point = JointTrajectoryPoint()

        point.positions = angles

        msg.points.append(point)
        msg.header.frame_id = "world" 
        msg.joint_names = ["short_arm1_joint", "short_arm2_joint", "short_arm3_joint", "short_arm4_joint", "short_arm5_joint", "short_arm6_joint"]

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
