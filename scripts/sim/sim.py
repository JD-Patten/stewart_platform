import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity, GetModelList
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose, Point, Quaternion
from rosgraph_msgs.msg import Clock
import tkinter as tk
import time
import math
import numpy as np

class EntityHandlerNode(Node):
    def __init__(self):
        super().__init__('entity_handler_node')
        #create publisher to end_effector_pose
        self.pub = self.create_publisher(Pose, "end_effector_pose", 1) 

        
        #create the clients for calling services from gazebo

        self.spawn_entity_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_entity_client = self.create_client(DeleteEntity, '/delete_entity')
        self.reset_sim_client = self.create_client(Empty, '/reset_simulation')
        self.get_model_list_client = self.create_client(GetModelList, '/get_model_list')

        while not self.spawn_entity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the /spawn_entity service to be available...')

        while not self.delete_entity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the /delete_entity service to be available...')

                                # amplitude, period, position shift, time shift
        self.default_parameters=   [[0,         0,          0,          0, ],   #x
                                    [0,         0,          0,          0, ],   #y
                                    [0,         0,        230,          0, ],   #z
                                    [0,         0,          0,          0, ],   #pitch
                                    [0,         0,          0,          0, ],   #roll
                                    [0,         0,          0,          0, ]]   #yaw
        
        self.parameters=           [[0,         5,          0,          0, ],   #x
                                    [0,         5,          0,          0, ],   #y
                                    [10,        5,        240,          0, ],   #z
                                    [0,         5,          0,          0, ],   #pitch
                                    [0,         5,          0,          0, ],   #roll
                                    [0,         5,          0,          0, ]]   #yaw
        #spawn first robot
        self.spawn_entity()

        #used to measure since robot was spawned
        self.start_time = 0.0

        #setup clock subscription to get simulation time
        qos_profile = rclpy.qos.QoSProfile(
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1,  # Set the depth according to your requirements
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE
        )
        self.clock_sub = self.create_subscription(Clock, 'clock', self.clock_sub_callback, qos_profile=qos_profile)

    def spawn_entity(self):     

        #gets urdf that was precompiled from the robot.urdf.xacro file
        with open('/home/jd/Workspaces/robot_ws/src/robot/urdf/robot.urdf', 'r') as file:
            robot_description = file.read()

        #sets up request for spawning bot
        request = SpawnEntity.Request()
        request.name = "bot"
        request.xml = robot_description

        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = 0.0
        request.initial_pose = pose

        future = self.spawn_entity_client.call_async(request)
        
    def delete_entity(self):

        request = DeleteEntity.Request()
        request.name = "bot"

        future = self.delete_entity_client.call_async(request)

    def reset_sim(self):
        request = Empty.Request()

        future = self.reset_sim_client.call_async(request)

        self.simulation_reset = True
    
    def get_model_list(self):
        request = GetModelList.Request()
        future = self.get_model_list_client.call_async(request)
            
        if future.result() is not None:
            model_names = future.result().model_names
            return model_names
        else:
            return []

    def publish_values(self, values):       #Values is list of 6 numbers first 3 in mm last 3 in degrees
        
        x = values[0] * 0.001               #converts to meters
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

    def clock_sub_callback(self, msg):
        seconds = msg.clock.sec 
        nanoseconds = msg.clock.nanosec
        time = seconds + nanoseconds * 0.000000001
        time_alive = time - self.start_time
        print(time_alive)

        #set values for end effector pose
        values = []

        for dof in range(6):
            amplitude =         self.parameters[dof][0]
            period =            self.parameters[dof][1]
            position_shift =    self.parameters[dof][2]
            time_shift =        self.parameters[dof][3]

            value = amplitude * math.cos((math.pi * 2 / period) * (time_alive + time_shift)) + position_shift
            values.append(value)
        #publish angles
        self.publish_values(values)

        if time_alive > 10:
            self.delete_entity()

        if time_alive >15:
            self.start_time = time
            self.spawn_entity()


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
    rclpy.init()           
    my_node = EntityHandlerNode()      

    try:
        rclpy.spin(my_node)              
    except KeyboardInterrupt:
        print("Terminating Node... ")
        my_node.destroy_node()

if __name__ == '__main__':
    main()
