#launch file for seeing the urdf model move around with the gui

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

    #runs the inverse kinematics node to calculate the servo angles for the current end effector position
        ExecuteProcess(
            cmd=['ros2', 'run', 'robot', 'inverse_kinematics_with_long_arms.py'],
            output="screen"
        ),

    #runs rviz2 for visualizing the robot
        ExecuteProcess(
            cmd=['ros2', 'run', 'rviz2', 'rviz2'],
            output="screen"
        ),

    #launches a gui for moving the end effector around
        ExecuteProcess(
            cmd=['ros2', 'run', 'robot', 'gui2.py'],
            output="screen"
        )
    ])
