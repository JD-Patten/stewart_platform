#!/usr/bin/env python3


from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

default_parent_frame = 'end_effector'
default_child_frame = 'arm1_end_connection'

default_x = 0.0
default_y = 0.0
default_z = 0.0

class FixedFrameBroadcaster(Node):
    def __init__(self):
        super().__init__('fixed_frame_tf2_broadcaster')

        self.declare_parameter('parent_frame', default_parent_frame)
        self.declare_parameter('child_frame', default_child_frame)
        self.declare_parameter('x_translation', default_x)
        self.declare_parameter('y_translation', default_y)
        self.declare_parameter('z_translation', default_z)

        self.parent_frame = self.get_parameter('parent_frame').get_parameter_value().string_value
        self.child_frame = self.get_parameter('child_frame').get_parameter_value().string_value
        
        self.x = self.get_parameter('x_translation').get_parameter_value().double_value 
        self.y = self.get_parameter('y_translation').get_parameter_value().double_value
        self.z = self.get_parameter('z_translation').get_parameter_value().double_value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.03, self.broadcast_timer_callback)




    def broadcast_timer_callback(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = self.z
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = FixedFrameBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()