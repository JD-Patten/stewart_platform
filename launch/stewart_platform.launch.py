#launch file for seeing the urdf model move around with the gui

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([

    #builds the urdf file and uses it in the joint state publisher
        ExecuteProcess(
            cmd=['ros2', 'launch', 'stewart_platform', 'urdf.launch.py'],
            output="screen"
        ),

    #sets up all of the TF2 frames for the end effector
        ExecuteProcess(
            cmd=['ros2', 'launch', 'stewart_platform', 'end_effector.launch.py'],
            output="screen"
        ),

    #runs rviz2 for visualizing the robot
        Node( 
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output="screen"
        ),

    #Runs the Inverse kinematics node which looks at the end effector pose, and sets joint positions for the arms
        Node(
            package='stewart_platform',
            executable='inverse_kinematics.py',
            name='ik_node',
            output="screen"
        ),

    # run the GUI to controll the robot
         Node(
            package='stewart_platform',
            executable='gui.py',
            name='gui_node',
            output="screen"
        )

    ])
