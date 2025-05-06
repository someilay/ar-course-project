#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from math import sin, cos
import time


class PosePublisher(Node):
    """
    Publishes pose commands to the YumiUdwadiaController
    """

    def __init__(self):
        super().__init__("pose_publisher")
        self.declare_parameter("namespace", "yumi")
        self.declare_parameter("controller_name", "udwadia_controller")
        self.declare_parameter("publish_rate", 10.0)  # Hz

        namespace = self.get_parameter("namespace").get_parameter_value().string_value
        controller_name = (
            self.get_parameter("controller_name").get_parameter_value().string_value
        )
        publish_rate = (
            self.get_parameter("publish_rate").get_parameter_value().double_value
        )

        # Create the publisher
        topic_name = f"/{namespace}/{controller_name}/command"
        self.publisher = self.create_publisher(PoseStamped, topic_name, 10)

        # Create a timer for regular publishing
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

        # Initialize time for creating trajectory
        self.start_time = time.time()

        self.get_logger().info(f"Publishing poses to {topic_name} at {publish_rate} Hz")

    def timer_callback(self):
        """Publishes a PoseStamped message based on simple trajectory"""
        current_time = time.time()
        elapsed_time = current_time - self.start_time

        # Create a simple circular trajectory
        radius = 0.1  # 10 cm radius
        frequency = 0.1  # Complete the circle in 10 seconds
        angle = 2.0 * 3.1415926 * frequency * elapsed_time

        # Create the message
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"

        # Set the position (moving in a circle)
        msg.pose.position.x = radius * cos(angle)  # X axis
        msg.pose.position.y = radius * sin(angle)  # Y axis
        msg.pose.position.z = 0.5  # Fixed height

        # Set the orientation (fixed orientation, w=1 is identity quaternion)
        msg.pose.orientation.w = 1.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0

        # Publish the message
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
