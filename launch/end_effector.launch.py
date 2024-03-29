from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

import math


draftAngle = math.radians(-20) #degrees to radians
servoHeight = 44 * .001       # mm to m

end_effector_arm_connections = [[-0.03464, 0.02, 0.0],
                                [0.0, 0.04, 0.0],
                                [0.03464, 0.02, 0.0],
                                [0.03464, -0.02, 0.0],
                                [0.0, -0.04, 0.0],
                                [-0.03464, -0.02, 0.0]]

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='robot',
            executable='end_effector_tf.py',    
            name='end_effector_tf'
        ),      

        Node(
            package='robot',
            executable='fixed_frame_broadcaster.py',
            name='arm_1_tf_frame_publisher',
            parameters=[ {"parent_frame": 'end_effector'}, {"child_frame": 'arm1_end_connection'}, {"x_translation": end_effector_arm_connections[0][0]}, {"y_translation": end_effector_arm_connections[0][1]}]
        ),

        Node(package='robot',
            executable='fixed_frame_broadcaster.py',
            name='arm_2_tf_frame_publisher',
            parameters=[ {"parent_frame": 'end_effector'}, {"child_frame": 'arm2_end_connection'}, {"x_translation": end_effector_arm_connections[1][0]}, {"y_translation": end_effector_arm_connections[1][1]}]
        ),

        Node(package='robot',
            executable='fixed_frame_broadcaster.py',
            name='arm_3_tf_frame_publisher',
            parameters=[ {"parent_frame": 'end_effector'}, {"child_frame": 'arm3_end_connection'}, {"x_translation": end_effector_arm_connections[2][0]}, {"y_translation": end_effector_arm_connections[2][1]}]
        ),
        
        Node(package='robot',
            executable='fixed_frame_broadcaster.py',
            name='arm_4_tf_frame_publisher',
            parameters=[ {"parent_frame": 'end_effector'}, {"child_frame": 'arm4_end_connection'}, {"x_translation": end_effector_arm_connections[3][0]}, {"y_translation": end_effector_arm_connections[3][1]}]
        ),

        Node(package='robot',
            executable='fixed_frame_broadcaster.py',
            name='arm_5_tf_frame_publisher',
            parameters=[ {"parent_frame": 'end_effector'}, {"child_frame": 'arm5_end_connection'}, {"x_translation": end_effector_arm_connections[4][0]}, {"y_translation": end_effector_arm_connections[4][1]}]
        ),
        
        Node(package='robot',
            executable='fixed_frame_broadcaster.py',
            name='arm_6_tf_frame_publisher',
            parameters=[ {"parent_frame": 'end_effector'}, {"child_frame": 'arm6_end_connection'}, {"x_translation": end_effector_arm_connections[5][0]}, {"y_translation": end_effector_arm_connections[5][1]}]
        ),        



        Node(
            package='rviz2',
            executable='rviz2',
        )
    
    ])
