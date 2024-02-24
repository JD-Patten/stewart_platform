#launch file for running the robot in the gazebo simulator

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        
    #builds the urdf file and uses it in the joint state publisher
        ExecuteProcess(
            cmd=['ros2', 'launch', 'robot', 'urdf.launch.py'],
            output="screen"
        ),

    #sets up all of the TF2 frames for the end effector
        ExecuteProcess(
            cmd=['ros2', 'launch', 'robot', 'end_effector.launch.py'],
            output="screen"
        ),

    #Inverse kinematics calculator for use in simulation
        ExecuteProcess(
            cmd=['ros2', 'run', 'robot', 'gazebo_servo_angle_calculator.py'],
            output="screen"
        ),

    #launches gazebo as the simulator
        ExecuteProcess(
            cmd=['ros2', 'launch', 'gazebo_ros', 'gazebo.launch.py', 'gui:=true'],
            output="screen"
        )
    ])

