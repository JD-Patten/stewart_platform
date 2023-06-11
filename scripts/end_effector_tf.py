#!/usr/bin/env python3

import rclpy                    
from rclpy.node import Node     
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster



class EndEffectorFramePublisher(Node):
    def __init__(self):
        super().__init__("end_effector_frame_publisher") #creates the node and names it
        self.sub = self.create_subscription(Pose,"end_effector_pose", self.handle_pose, 1) 
        self.tf_broadcaster = TransformBroadcaster(self)

    def handle_pose(self, msg):
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'end_effector'

        t.transform.translation.x = msg.position.x
        t.transform.translation.y = msg.position.y
        t.transform.translation.z = msg.position.z

        t.transform.rotation.x = msg.orientation.x
        t.transform.rotation.y = msg.orientation.y
        t.transform.rotation.z = msg.orientation.z
        t.transform.rotation.w = msg.orientation.w

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init()                        
    node = EndEffectorFramePublisher()      
    print("Waiting for pose to be published...")

    try:
        rclpy.spin(node)              #Starts the node and keeps it running untill it's interrupted by the user
    except KeyboardInterrupt:
        print("Terminating Node... ")
        node.destroy_node()


if __name__ == '__main__':
    main()