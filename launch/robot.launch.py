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
            cmd=['ros2', 'run', 'robot', 'inverse_kinematics_with_long_arms.py'],
            output="screen"
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'rviz2', 'rviz2'],
            output="screen"
        ),

        ExecuteProcess(
            cmd=['ros2', 'run', 'robot', 'gui2.py'],
            output="screen"
        )
    ])
