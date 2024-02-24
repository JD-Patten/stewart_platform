# This is the launch file for the end effector TF2 frames.
# It launches a dynamic frame for the center of the end effector 
# and 6 static frames at the arm connection points on the end effector


from launch import LaunchDescription
from launch_ros.actions import Node


#x,y,z values for the connection points of the arms on the end effector

                                #x            y           z   
end_effector_arm_connections = [[-0.03153,    0.02461,    -0.0125], 
                                [-0.00555,    0.03961,    -0.0125],
                                [0.03781,     0.015,      -0.0125],
                                [0.03781,    -0.015,      -0.0125],
                                [-0.00555,   -0.03961,    -0.0125],
                                [-0.03153,   -0.02461,    -0.0125]]

def generate_launch_description():
    return LaunchDescription([
    

        #launch the node to set up a dynamic frame for the center of the end effector

        Node(
            package='robot',
            executable='end_effector_tf.py',    
            name='end_effector_tf'
        ),


        #launch 6 more nodes, each publishing a fixed frame at a connection point of one of the arms  and the top plate
        
        Node(
            package='robot',
            executable='fixed_frame_broadcaster.py',
            name='arm_1_tf_frame_publisher',
            parameters=[ {"parent_frame": 'end_effector'}, {"child_frame": 'arm1_end_connection'}, {"x_translation": end_effector_arm_connections[0][0]}, {"y_translation": end_effector_arm_connections[0][1]}, {"z_translation": end_effector_arm_connections[0][2]}]
        ),

        Node(package='robot',
            executable='fixed_frame_broadcaster.py',
            name='arm_2_tf_frame_publisher',
            parameters=[ {"parent_frame": 'end_effector'}, {"child_frame": 'arm2_end_connection'}, {"x_translation": end_effector_arm_connections[1][0]}, {"y_translation": end_effector_arm_connections[1][1]}, {"z_translation": end_effector_arm_connections[1][2]}]
        ),

        Node(package='robot',
            executable='fixed_frame_broadcaster.py',
            name='arm_3_tf_frame_publisher',
            parameters=[ {"parent_frame": 'end_effector'}, {"child_frame": 'arm3_end_connection'}, {"x_translation": end_effector_arm_connections[2][0]}, {"y_translation": end_effector_arm_connections[2][1]}, {"z_translation": end_effector_arm_connections[2][2]}]
        ),
        
        Node(package='robot',
            executable='fixed_frame_broadcaster.py',
            name='arm_4_tf_frame_publisher',
            parameters=[ {"parent_frame": 'end_effector'}, {"child_frame": 'arm4_end_connection'}, {"x_translation": end_effector_arm_connections[3][0]}, {"y_translation": end_effector_arm_connections[3][1]}, {"z_translation": end_effector_arm_connections[3][2]}]
        ),

        Node(package='robot',
            executable='fixed_frame_broadcaster.py',
            name='arm_5_tf_frame_publisher',
            parameters=[ {"parent_frame": 'end_effector'}, {"child_frame": 'arm5_end_connection'}, {"x_translation": end_effector_arm_connections[4][0]}, {"y_translation": end_effector_arm_connections[4][1]}, {"z_translation": end_effector_arm_connections[4][2]}]
        ),
        
        Node(package='robot',
            executable='fixed_frame_broadcaster.py',
            name='arm_6_tf_frame_publisher',
            parameters=[ {"parent_frame": 'end_effector'}, {"child_frame": 'arm6_end_connection'}, {"x_translation": end_effector_arm_connections[5][0]}, {"y_translation": end_effector_arm_connections[5][1]}, {"z_translation": end_effector_arm_connections[5][2]}]
        ),        

    
    ])
