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
            cmd=['ros2', 'launch', 'gazebo_ros', 'gazebo.launch.py', 'world:=src/robot_testing/worlds/world6.world'],
            output="screen"
        )
        #,
       # ExecuteProcess(
        #    cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py', '-topic', 'robot_description', '-entity', 'some_name'],
        #    output="screen"
        #),
    ])

