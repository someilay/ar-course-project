import sys
import rclpy
import threading
import numpy as np
import signal
import yaml
import os
import re
import math
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped, AccelStamped
from moveit_msgs.msg import CartesianTrajectoryPoint
from python_qt_binding.binding_helper import loadUi

# from python_qt_binding.QtWidgets import (
#     QApplication,
#     QWidget,
#     QVBoxLayout,
#     QHBoxLayout,
#     QLabel,
#     QDoubleSpinBox,
#     QPushButton,
#     QTabWidget,
#     QGridLayout,
#     QGroupBox,
# )
# from python_qt_binding.QtCore import Qt
from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QDoubleSpinBox,
    QPushButton,
    QTabWidget,
    QGridLayout,
    QGroupBox,
)
from PyQt5.QtCore import Qt


class ClosableQWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.is_running = True

    def closeEvent(self, event):
        self.is_running = False
        event.accept()


class YumiUdwadiaGui(Node):
    def __init__(self, app: QApplication):
        super().__init__("yumi_udwadia_gui")

        # Default initial positions
        self.default_config = {
            "position": {"x": 0.6, "y": -0.15, "z": 0.5},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        }

        # Declare parameters for topic names and config file
        self.declare_parameter("target_pose_topic", "command")
        self.declare_parameter("debug_cartesian_topic", "debug/cartesian")
        self.declare_parameter("config_file_path", "controller.yaml")

        # Get parameters
        target_pose_topic = (
            self.get_parameter("target_pose_topic").get_parameter_value().string_value
        )
        debug_cartesian_topic = (
            self.get_parameter("debug_cartesian_topic")
            .get_parameter_value()
            .string_value
        )
        config_file_path = (
            self.get_parameter("config_file_path").get_parameter_value().string_value
        )

        # Log the topic names
        self.get_logger().info(f"Target pose topic: {target_pose_topic}")
        self.get_logger().info(f"Debug cartesian topic: {debug_cartesian_topic}")

        # Load configurations from YAML file if provided
        self.config = self.default_config
        if config_file_path:
            self.load_config_from_yaml(config_file_path)

        # Create publishers for pose, velocity, and acceleration commands
        self.pose_pub = self.create_publisher(PoseStamped, target_pose_topic, 10)

        # Create subscription for actual cartesian position
        self.actual_pose_sub = self.create_subscription(
            CartesianTrajectoryPoint,
            debug_cartesian_topic,
            self.actual_position_callback,
            10,
        )

        # Store the actual position
        self.actual_position = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": 0.0,
            "qw": 1.0,
        }

        # Initialize the GUI
        self.app = app
        self.init_gui()

    def euler_to_quaternion(
        self, roll: float, pitch: float, yaw: float
    ) -> tuple[float, float, float, float]:
        """Convert euler angles to quaternion."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        # Normalize the quaternion
        arr = np.array([qx, qy, qz, qw])
        arr /= np.linalg.norm(arr)
        return tuple(arr)

    def load_config_from_yaml(self, file_path: str):
        """Load configuration from a YAML file.

        Supports two formats:
        1. Simple format with direct position and orientation keys
        2. udwadia_controller.yaml.tmp format with ros__parameters structure and placeholders
        """
        try:
            if not os.path.exists(file_path):
                self.get_logger().error(f"Config file not found: {file_path}")
                return

            # First try to read the file directly
            with open(file_path, "r") as file:
                file_content = file.read()

            yaml_config = yaml.safe_load(file_content)
            self._parse_simple_yaml(yaml_config)
        except Exception as e:
            self.get_logger().error(f"Error loading config file: {str(e)}")

    def _parse_simple_yaml(self, yaml_config: dict):
        """Parse a simple YAML format with direct position and orientation keys."""
        names = [i for i in yaml_config.keys() if "udwadia_controller" in i]
        if not names:
            self.get_logger().error("No udwadia_controller found in the yaml file")
            return

        controller_config = yaml_config[names[0]]
        x = controller_config.get(
            "initial_position_x", self.default_config["position"]["x"]
        )
        y = controller_config.get(
            "initial_position_y", self.default_config["position"]["y"]
        )
        z = controller_config.get(
            "initial_position_z", self.default_config["position"]["z"]
        )

        roll = controller_config.get("initial_orientation_roll", 0)
        pitch = controller_config.get("initial_orientation_pitch", 0)
        yaw = controller_config.get("initial_orientation_yaw", 0)
        qx, qy, qz, qw = self.euler_to_quaternion(roll, pitch, yaw)

        self.config = {
            "position": {
                "x": x,
                "y": y,
                "z": z,
            },
            "orientation": {
                "x": qx,
                "y": qy,
                "z": qz,
                "w": qw,
            },
        }

    def actual_position_callback(self, msg):
        # Update the stored actual position when a message is received
        self.actual_position["x"] = msg.point.pose.position.x
        self.actual_position["y"] = msg.point.pose.position.y
        self.actual_position["z"] = msg.point.pose.position.z
        self.actual_position["qx"] = msg.point.pose.orientation.x
        self.actual_position["qy"] = msg.point.pose.orientation.y
        self.actual_position["qz"] = msg.point.pose.orientation.z
        self.actual_position["qw"] = msg.point.pose.orientation.w

        # Update the display
        self.update_actual_position_display()

    def update_actual_position_display(self):
        # Update the label values with the current actual position
        self.actual_pos_x_label.setText(f"{self.actual_position['x']:.6f}")
        self.actual_pos_y_label.setText(f"{self.actual_position['y']:.6f}")
        self.actual_pos_z_label.setText(f"{self.actual_position['z']:.6f}")
        self.actual_quat_x_label.setText(f"{self.actual_position['qx']:.6f}")
        self.actual_quat_y_label.setText(f"{self.actual_position['qy']:.6f}")
        self.actual_quat_z_label.setText(f"{self.actual_position['qz']:.6f}")
        self.actual_quat_w_label.setText(f"{self.actual_position['qw']:.6f}")

    def init_gui(self):
        self.window = ClosableQWidget()
        self.window.setWindowTitle("YuMi Udwadia Controller GUI")
        self.window.resize(800, 600)

        # Create the main layout
        main_layout = QVBoxLayout(self.window)

        # Create tabs for pose, velocity, and acceleration
        tabs = QTabWidget()
        main_layout.addWidget(tabs)

        # Create tab widgets
        pose_widget = self.create_pose_widget()

        # Add tabs
        tabs.addTab(pose_widget, "Pose")

        # Create the send and reset buttons
        button_layout = QHBoxLayout()
        self.send_button = QPushButton("Send Command")
        self.send_button.clicked.connect(self.send_command)
        self.reset_button = QPushButton("Reset")
        self.reset_button.clicked.connect(self.reset_values)

        button_layout.addWidget(self.send_button)
        button_layout.addWidget(self.reset_button)
        main_layout.addLayout(button_layout)

        self.window.setLayout(main_layout)
        self.window.show()

    def create_pose_widget(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)

        # Create position group
        position_group = QGroupBox("Position")
        position_layout = QGridLayout()

        # Position inputs (x, y, z) - using values from config
        pos = self.config["position"]
        self.pos_x = self.create_double_spinbox(-2.0, 2.0, 0.001, pos["x"])
        self.pos_y = self.create_double_spinbox(-2.0, 2.0, 0.001, pos["y"])
        self.pos_z = self.create_double_spinbox(-2.0, 2.0, 0.001, pos["z"])

        position_layout.addWidget(QLabel("X:"), 0, 0)
        position_layout.addWidget(self.pos_x, 0, 1)
        position_layout.addWidget(QLabel("Y:"), 1, 0)
        position_layout.addWidget(self.pos_y, 1, 1)
        position_layout.addWidget(QLabel("Z:"), 2, 0)
        position_layout.addWidget(self.pos_z, 2, 1)

        # Add actual position labels
        position_layout.addWidget(QLabel("Actual X:"), 0, 2)
        self.actual_pos_x_label = QLabel("0.000000")
        position_layout.addWidget(self.actual_pos_x_label, 0, 3)

        position_layout.addWidget(QLabel("Actual Y:"), 1, 2)
        self.actual_pos_y_label = QLabel("0.000000")
        position_layout.addWidget(self.actual_pos_y_label, 1, 3)

        position_layout.addWidget(QLabel("Actual Z:"), 2, 2)
        self.actual_pos_z_label = QLabel("0.000000")
        position_layout.addWidget(self.actual_pos_z_label, 2, 3)

        position_group.setLayout(position_layout)
        layout.addWidget(position_group)

        # Create orientation group
        orientation_group = QGroupBox("Orientation (Quaternion)")
        orientation_layout = QGridLayout()

        # Orientation inputs (quaternion x, y, z, w) - using values from config
        ori = self.config["orientation"]
        self.quat_x = self.create_double_spinbox(-1.0, 1.0, 0.001, ori["x"])
        self.quat_y = self.create_double_spinbox(-1.0, 1.0, 0.001, ori["y"])
        self.quat_z = self.create_double_spinbox(-1.0, 1.0, 0.001, ori["z"])
        self.quat_w = self.create_double_spinbox(-1.0, 1.0, 0.001, ori["w"])

        orientation_layout.addWidget(QLabel("X:"), 0, 0)
        orientation_layout.addWidget(self.quat_x, 0, 1)
        orientation_layout.addWidget(QLabel("Y:"), 1, 0)
        orientation_layout.addWidget(self.quat_y, 1, 1)
        orientation_layout.addWidget(QLabel("Z:"), 2, 0)
        orientation_layout.addWidget(self.quat_z, 2, 1)
        orientation_layout.addWidget(QLabel("W:"), 3, 0)
        orientation_layout.addWidget(self.quat_w, 3, 1)

        # Add actual orientation labels
        orientation_layout.addWidget(QLabel("Actual X:"), 0, 2)
        self.actual_quat_x_label = QLabel("0.000000")
        orientation_layout.addWidget(self.actual_quat_x_label, 0, 3)

        orientation_layout.addWidget(QLabel("Actual Y:"), 1, 2)
        self.actual_quat_y_label = QLabel("0.000000")
        orientation_layout.addWidget(self.actual_quat_y_label, 1, 3)

        orientation_layout.addWidget(QLabel("Actual Z:"), 2, 2)
        self.actual_quat_z_label = QLabel("0.000000")
        orientation_layout.addWidget(self.actual_quat_z_label, 2, 3)

        orientation_layout.addWidget(QLabel("Actual W:"), 3, 2)
        self.actual_quat_w_label = QLabel("1.000000")
        orientation_layout.addWidget(self.actual_quat_w_label, 3, 3)

        orientation_group.setLayout(orientation_layout)
        layout.addWidget(orientation_group)

        return widget

    def create_double_spinbox(self, min_val, max_val, step, default_val):
        spinbox = QDoubleSpinBox()
        spinbox.setRange(min_val, max_val)
        spinbox.setSingleStep(step)
        spinbox.setValue(default_val)
        spinbox.setDecimals(4)
        return spinbox

    def send_command(self):
        # Create and publish pose command
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "world"

        # Set position
        pose_msg.pose.position.x = self.pos_x.value()
        pose_msg.pose.position.y = self.pos_y.value()
        pose_msg.pose.position.z = self.pos_z.value()

        # Set orientation (quaternion)
        arr = np.array(
            [
                self.quat_x.value(),
                self.quat_y.value(),
                self.quat_z.value(),
                self.quat_w.value(),
            ]
        )

        if np.linalg.norm(arr) < 1e-4:
            self.get_logger().warn("Singular orientation")
            return

        arr = arr / np.linalg.norm(arr)
        pose_msg.pose.orientation.x = arr[0]
        pose_msg.pose.orientation.y = arr[1]
        pose_msg.pose.orientation.z = arr[2]
        pose_msg.pose.orientation.w = arr[3]

        # Publish the messages
        self.pose_pub.publish(pose_msg)

        self.get_logger().info("Commands sent to YumiUdwadiaController")

    def reset_values(self):
        # Reset pose values to the loaded configuration
        pos = self.config["position"]
        ori = self.config["orientation"]

        self.pos_x.setValue(pos["x"])
        self.pos_y.setValue(pos["y"])
        self.pos_z.setValue(pos["z"])
        self.quat_x.setValue(ori["x"])
        self.quat_y.setValue(ori["y"])
        self.quat_z.setValue(ori["z"])
        self.quat_w.setValue(ori["w"])

        self.get_logger().info("Values reset to configuration")

    def run(self):
        # Process any pending ROS events
        while self.window.is_running:
            rclpy.spin_once(self, timeout_sec=0.1)


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    gui = YumiUdwadiaGui(app)

    threading.Thread(target=gui.run).start()
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
