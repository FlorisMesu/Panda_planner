#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time


class FakeVisionPublisher(Node):
    def __init__(self):
        super().__init__('fake_vision_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/target_pose', 10)
        timer_period = 5.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_pose)
        self.get_logger().info("Fake Vision Publisher started.")

    def publish_pose(self):
        msg = PoseStamped()
        msg.header.frame_id = "world"  # or "base_link", depending on your TF tree
        msg.header.stamp = self.get_clock().now().to_msg()

        # Simulated pose
        msg.pose.position.x = 0.3
        msg.pose.position.y = 0.2
        msg.pose.position.z = 0.1
        msg.pose.orientation.w = 1.0  # No rotation

        self.publisher_.publish(msg)
        self.get_logger().info(f"Published target pose: {msg.pose.position}")


def main(args=None):
    rclpy.init(args=args)
    node = FakeVisionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
