#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import (
    CartesianTrajectory,
    CartesianTrajectoryPoint,
    CartesianPoint,
)
from geometry_msgs.msg import Pose, Twist, Accel
from builtin_interfaces.msg import Duration
from math import sin, cos
import time


class CartesianTrajectoryPublisher(Node):
    """
    Publishes Cartesian trajectory commands to the YumiUdwadiaController
    """

    def __init__(self):
        super().__init__("cartesian_trajectory_publisher")
        self.declare_parameter("namespace", "yumi")
        self.declare_parameter("controller_name", "udwadia_controller")

        namespace = self.get_parameter("namespace").get_parameter_value().string_value
        controller_name = (
            self.get_parameter("controller_name").get_parameter_value().string_value
        )

        # Create the publisher
        topic_name = f"/{namespace}/{controller_name}/trajectory"
        self.publisher = self.create_publisher(CartesianTrajectory, topic_name, 10)

        # Wait a bit for the publisher to be ready
        time.sleep(1.0)

        # Publish a sample trajectory
        self.publish_sample_trajectory()

        self.get_logger().info(f"Published sample trajectory to {topic_name}")

    def publish_sample_trajectory(self):
        """Creates and publishes a sample Cartesian trajectory"""
        trajectory = CartesianTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.header.frame_id = "world"

        # Define the trajectory points (square motion)
        points = []

        # Create 4 corners of a square at fixed height
        square_size = 0.1  # 10 cm square
        positions = [
            (square_size, square_size, 0.5),  # Top right
            (-square_size, square_size, 0.5),  # Top left
            (-square_size, -square_size, 0.5),  # Bottom left
            (square_size, -square_size, 0.5),  # Bottom right
            (square_size, square_size, 0.5),  # Back to top right to complete the square
        ]

        # Create each trajectory point with appropriate timing
        for i, (x, y, z) in enumerate(positions):
            point = CartesianTrajectoryPoint()

            # Set the pose
            point.point = CartesianPoint()
            point.point.pose.position.x = x
            point.point.pose.position.y = y
            point.point.pose.position.z = z
            point.point.pose.orientation.w = 1.0  # Identity quaternion

            # Set zero velocity for simplicity
            point.point.velocity.linear.x = 0.0
            point.point.velocity.linear.y = 0.0
            point.point.velocity.linear.z = 0.0

            # Set zero acceleration for simplicity
            point.point.acceleration.linear.x = 0.0
            point.point.acceleration.linear.y = 0.0
            point.point.acceleration.linear.z = 0.0

            # Set time from start (2 seconds per segment)
            point.time_from_start = Duration(sec=i * 2, nanosec=0)
            points.append(point)

        trajectory.points = points

        # Publish the trajectory
        self.publisher.publish(trajectory)


def main(args=None):
    rclpy.init(args=args)
    node = CartesianTrajectoryPublisher()
    try:
        rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
