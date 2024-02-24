from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        
        ExecuteProcess(
            cmd=['ros2', 'launch', 'robot', 'urdf.launch.py'],
            output="screen"
        ),
        ExecuteProcess(
            cmd=['ros2', 'launch', 'robot', 'end_effector.launch.py'],
            output="screen"
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'robot', 'gazebo_servo_angle_calculator.py'],
            output="screen"
        ),

        ExecuteProcess(
            cmd=['ros2', 'launch', 'gazebo_ros', 'gazebo.launch.py', 'gui:=true'],
            output="screen"
        )
    ])

