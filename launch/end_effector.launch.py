

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
    nodes = []

    # Dynamic frame for the center of the end effector
    nodes.append(Node(
        package='stewart_platform',
        executable='end_effector_tf.py',
        name='end_effector_tf'
    ))

    # Static frames for each arm connection point
    for i, position in enumerate(end_effector_arm_connections):
        nodes.append(Node(
            package='stewart_platform',
            executable='fixed_frame_broadcaster.py',
            name=f'arm_{i+1}_tf_frame_publisher',
            parameters=[
                {"parent_frame": 'end_effector'},
                {"child_frame": f'arm{i+1}_end_connection'},
                {"x_translation": position[0]},
                {"y_translation": position[1]},
                {"z_translation": position[2]}
            ]
        ))

    return LaunchDescription(nodes)