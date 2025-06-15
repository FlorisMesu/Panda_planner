#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class FakeVisionPublisherAlternating(Node):
    def __init__(self):
        super().__init__('fake_vision_publisher_alternating')
        self.publisher_ = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.timer_period = 5.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.publish_pose)
        self.pose_toggle = True  # Toggle flag to switch between A and B
        self.get_logger().info("Fake Vision Publisher started.")

    def publish_pose(self):
        msg = PoseStamped()
        msg.header.frame_id = "base_link"
        msg.header.stamp = self.get_clock().now().to_msg()

        if self.pose_toggle:
            # Pose A
            msg.pose.position.x = 0.4
            msg.pose.position.y = 0.0
            msg.pose.position.z = 0.3
            self.get_logger().info("Publishing Pose A")
        else:
            # Pose B
            msg.pose.position.x = 0.5
            msg.pose.position.y = 0.0
            msg.pose.position.z = 0.8
            self.get_logger().info("Publishing Pose B")

        msg.pose.orientation.w = 1.0  # No rotation
        self.publisher_.publish(msg)

        # Toggle for next time
        self.pose_toggle = not self.pose_toggle


def main(args=None):
    rclpy.init(args=args)
    node = FakeVisionPublisherAlternating()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
