#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster


class UwbOdomBroadcaster(Node):
    def __init__(self):
        super().__init__('uwb_odom_broadcaster')

        # subscribe to the UWB pose topic
        self.subscription = self.create_subscription(
            PoseStamped,
            '/uwb/pose',   # change if your topic name is different
            self.pose_callback,
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

    def pose_callback(self, msg: PoseStamped):
        # Convert PoseStamped into TF odom -> base_link
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id  # should be "odom"
        t.child_frame_id = 'base_link'

        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z

        t.transform.rotation = msg.pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = UwbOdomBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

