import sys
import rclpy
import threading
import numpy as np
import signal
import yaml
import os
import re
import math
import time

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped, AccelStamped
from moveit_msgs.msg import CartesianTrajectoryPoint, CartesianTrajectory
from scipy.spatial.transform import Rotation as Rot

from python_qt_binding.binding_helper import loadUi
from python_qt_binding.QtWidgets import (
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
    QSlider,
    QCheckBox,
    QComboBox,
    QSpinBox,
)
from python_qt_binding.QtCore import Qt, QTimer


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
        self.default_config: dict[str, dict[str, float]] = {
            "position": {"x": 0.6, "y": -0.15, "z": 0.5},
            "orientation": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
        }

        # Define position slider ranges
        self.position_ranges: dict[str, dict[str, float]] = {
            "x": {"min": 0.0, "max": 1.0},  # Forward/backward
            "y": {"min": -0.5, "max": 0.5},  # Left/right
            "z": {"min": 0.0, "max": 1.0},  # Up/down
        }

        # Flag for real-time updates
        self.realtime_updates_enabled: bool = True

        # Flag to track if a trajectory is currently being sent
        self.is_sending_trajectory: bool = False

        # Number of points in the trajectory
        self.num_points: int = 50

        # Declare parameters for topic names and config file
        self.declare_parameter("target_pose_topic", "command")
        self.declare_parameter("debug_cartesian_topic", "debug/cartesian")
        self.declare_parameter("trajectory_topic", "trajectory")
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
        trajectory_topic = (
            self.get_parameter("trajectory_topic").get_parameter_value().string_value
        )
        config_file_path = (
            self.get_parameter("config_file_path").get_parameter_value().string_value
        )

        # Log the topic names
        self.get_logger().info(f"Target pose topic: {target_pose_topic}")
        self.get_logger().info(f"Debug cartesian topic: {debug_cartesian_topic}")
        self.get_logger().info(f"Trajectory topic: {trajectory_topic}")

        # Load configurations from YAML file if provided
        self.config: dict[str, dict[str, float]] = self.default_config.copy()
        if config_file_path:
            self.load_config_from_yaml(config_file_path)

        # Create publishers for pose, velocity, and acceleration commands
        self.pose_pub = self.create_publisher(PoseStamped, target_pose_topic, 10)

        # Create publisher for trajectory commands
        self.trajectory_pub = self.create_publisher(
            CartesianTrajectory, trajectory_topic, 10
        )

        # Create subscription for actual cartesian position
        self.actual_pose_sub = self.create_subscription(
            CartesianTrajectoryPoint,
            debug_cartesian_topic,
            self.actual_position_callback,
            10,
        )

        # Store the actual position
        self.actual_position: dict[str, float] = {
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
        rot = Rot.from_euler("xyz", [roll, pitch, yaw], degrees=True)
        return rot.as_quat()

    def quaternion_to_euler(
        self, qx: float, qy: float, qz: float, qw: float
    ) -> np.ndarray:
        """Convert quaternion to euler angles (roll, pitch, yaw)."""
        rot = Rot.from_quat(np.array([qx, qy, qz, qw]))
        rot_euler = rot.as_euler("xyz", degrees=True)
        return np.array(rot_euler)

    def load_config_from_yaml(self, file_path: str) -> None:
        """Load configuration from a YAML file."""
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

    def _parse_simple_yaml(self, yaml_config: dict[str, any]) -> None:
        """Parse a simple YAML format with direct position and orientation keys."""
        names = [i for i in yaml_config.keys() if "udwadia_controller" in i]
        if not names:
            self.get_logger().error("No udwadia_controller found in the yaml file")
            return

        controller_config = yaml_config[names[0]]

        if "ros__parameters" not in controller_config:
            self.get_logger().error("No ros__parameters found in the yaml file")
            return

        parameters = controller_config["ros__parameters"]

        x = parameters.get("initial_position_x", self.default_config["position"]["x"])
        y = parameters.get("initial_position_y", self.default_config["position"]["y"])
        z = parameters.get("initial_position_z", self.default_config["position"]["z"])

        roll = parameters.get("initial_orientation_roll", 0)
        pitch = parameters.get("initial_orientation_pitch", 0)
        yaw = parameters.get("initial_orientation_yaw", 0)

        self.config = {
            "position": {
                "x": x,
                "y": y,
                "z": z,
            },
            "orientation": {
                "roll": roll,
                "pitch": pitch,
                "yaw": yaw,
            },
        }

    def actual_position_callback(self, msg: CartesianTrajectoryPoint) -> None:
        """Update internal state when a new position message is received."""
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

    def update_actual_position_display(self) -> None:
        """Update the UI labels with the current robot position."""
        # Update the position label values
        self.actual_pos_x_label.setText(f"{self.actual_position['x']:.4f}")
        self.actual_pos_y_label.setText(f"{self.actual_position['y']:.4f}")
        self.actual_pos_z_label.setText(f"{self.actual_position['z']:.4f}")

        # Convert quaternion to euler angles for display
        roll, pitch, yaw = self.quaternion_to_euler(
            self.actual_position["qx"],
            self.actual_position["qy"],
            self.actual_position["qz"],
            self.actual_position["qw"],
        )

        # Display in degrees for better readability
        self.actual_roll_label.setText(f"{roll:.2f}°")
        self.actual_pitch_label.setText(f"{pitch:.2f}°")
        self.actual_yaw_label.setText(f"{yaw:.2f}°")

    def init_gui(self) -> None:
        """Initialize the GUI components."""
        self.window = ClosableQWidget()
        self.window.setWindowTitle("YuMi Udwadia Controller GUI")
        self.window.resize(900, 650)

        # Create the main layout
        main_layout = QVBoxLayout(self.window)
        main_layout.setContentsMargins(
            20, 20, 20, 20
        )  # Add margins (left, top, right, bottom)
        main_layout.setSpacing(15)  # Add spacing between elements

        # Create tabs for pose, velocity, and acceleration
        tabs = QTabWidget()
        main_layout.addWidget(tabs)

        # Create tab widgets
        pose_widget = self.create_pose_widget()
        trajectory_widget = self.create_trajectory_widget()

        # Add tabs
        tabs.addTab(pose_widget, "Pose")
        tabs.addTab(trajectory_widget, "Trajectory")

        # Create the real-time update checkbox
        realtime_layout = QHBoxLayout()
        self.realtime_checkbox = QCheckBox("Enable Real-time Updates")
        self.realtime_checkbox.setChecked(self.realtime_updates_enabled)
        self.realtime_checkbox.stateChanged.connect(self.toggle_realtime_updates)
        realtime_layout.addWidget(self.realtime_checkbox)
        main_layout.addLayout(realtime_layout)

        # Create the send and reset buttons
        button_layout = QHBoxLayout()
        self.send_button = QPushButton("Send Command")
        self.send_button.clicked.connect(self.send_command)
        self.reset_button = QPushButton("Reset")
        self.reset_button.clicked.connect(self.reset_values)

        button_layout.addWidget(self.send_button)
        button_layout.addWidget(self.reset_button)
        main_layout.addLayout(button_layout)

        # Add status message label
        self.status_label = QLabel("")
        main_layout.addWidget(self.status_label)

        self.window.setLayout(main_layout)
        self.window.show()

    def create_pose_widget(self) -> QWidget:
        """Create the position and orientation control widget."""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setSpacing(
            20
        )  # Add more space between position and orientation sections

        # Create position group with sliders
        position_group = QGroupBox("Position")
        position_layout = QGridLayout()

        # Set spacing for the grid layout
        position_layout.setHorizontalSpacing(10)
        position_layout.setVerticalSpacing(15)

        # Position sliders (x, y, z) - using values from config
        pos = self.config["position"]

        # Create position sliders with different ranges for each axis
        self.pos_x_slider = self.create_position_slider(
            "x",
            pos["x"],
            self.position_ranges["x"]["min"],
            self.position_ranges["x"]["max"],
        )
        self.pos_y_slider = self.create_position_slider(
            "y",
            pos["y"],
            self.position_ranges["y"]["min"],
            self.position_ranges["y"]["max"],
        )
        self.pos_z_slider = self.create_position_slider(
            "z",
            pos["z"],
            self.position_ranges["z"]["min"],
            self.position_ranges["z"]["max"],
        )

        # Labels to display current slider values
        self.pos_x_value_label = QLabel(f"{pos['x']:.4f}")
        self.pos_y_value_label = QLabel(f"{pos['y']:.4f}")
        self.pos_z_value_label = QLabel(f"{pos['z']:.4f}")

        # Connect slider value changes to update labels
        self.pos_x_slider.valueChanged.connect(
            lambda v: self.update_position_label("x", v, self.pos_x_value_label)
        )
        self.pos_y_slider.valueChanged.connect(
            lambda v: self.update_position_label("y", v, self.pos_y_value_label)
        )
        self.pos_z_slider.valueChanged.connect(
            lambda v: self.update_position_label("z", v, self.pos_z_value_label)
        )

        # Connect slider value changes to send commands in real-time
        self.pos_x_slider.valueChanged.connect(lambda: self.on_slider_changed())
        self.pos_y_slider.valueChanged.connect(lambda: self.on_slider_changed())
        self.pos_z_slider.valueChanged.connect(lambda: self.on_slider_changed())

        # Add sliders and labels to layout
        position_layout.addWidget(QLabel("X:"), 0, 0)
        position_layout.addWidget(self.pos_x_slider, 0, 1)
        position_layout.addWidget(self.pos_x_value_label, 0, 2)

        position_layout.addWidget(QLabel("Y:"), 1, 0)
        position_layout.addWidget(self.pos_y_slider, 1, 1)
        position_layout.addWidget(self.pos_y_value_label, 1, 2)

        position_layout.addWidget(QLabel("Z:"), 2, 0)
        position_layout.addWidget(self.pos_z_slider, 2, 1)
        position_layout.addWidget(self.pos_z_value_label, 2, 2)

        # Add actual position labels
        position_layout.addWidget(QLabel("Actual X:"), 0, 3)
        self.actual_pos_x_label = QLabel("0.0000")
        position_layout.addWidget(self.actual_pos_x_label, 0, 4)

        position_layout.addWidget(QLabel("Actual Y:"), 1, 3)
        self.actual_pos_y_label = QLabel("0.0000")
        position_layout.addWidget(self.actual_pos_y_label, 1, 4)

        position_layout.addWidget(QLabel("Actual Z:"), 2, 3)
        self.actual_pos_z_label = QLabel("0.0000")
        position_layout.addWidget(self.actual_pos_z_label, 2, 4)

        position_group.setLayout(position_layout)
        layout.addWidget(position_group)

        # Create orientation group (using Euler angles with sliders)
        orientation_group = QGroupBox("Orientation (Euler Angles)")
        orientation_layout = QGridLayout()

        # Set spacing for the grid layout
        orientation_layout.setHorizontalSpacing(10)
        orientation_layout.setVerticalSpacing(15)

        # Orientation inputs (roll, pitch, yaw) - using values from config and sliders
        ori = self.config["orientation"]

        # Create sliders for angles in degrees
        # Scale: -180 to 180 degrees mapped to slider range (0-360)
        self.roll_slider = self.create_angle_slider(ori["roll"])
        self.pitch_slider = self.create_angle_slider(ori["pitch"])
        self.yaw_slider = self.create_angle_slider(ori["yaw"])

        # Labels to display current slider values
        self.roll_value_label = QLabel(f"{ori['roll']:.1f}°")
        self.pitch_value_label = QLabel(f"{ori['pitch']:.1f}°")
        self.yaw_value_label = QLabel(f"{ori['yaw']:.1f}°")

        # Connect slider value changes to update labels
        self.roll_slider.valueChanged.connect(
            lambda v: self.update_angle_label(v, self.roll_value_label)
        )
        self.pitch_slider.valueChanged.connect(
            lambda v: self.update_angle_label(v, self.pitch_value_label)
        )
        self.yaw_slider.valueChanged.connect(
            lambda v: self.update_angle_label(v, self.yaw_value_label)
        )

        # Connect slider value changes to send commands in real-time
        self.roll_slider.valueChanged.connect(lambda: self.on_slider_changed())
        self.pitch_slider.valueChanged.connect(lambda: self.on_slider_changed())
        self.yaw_slider.valueChanged.connect(lambda: self.on_slider_changed())

        # Add sliders and labels to layout
        orientation_layout.addWidget(QLabel("Roll (X):"), 0, 0)
        orientation_layout.addWidget(self.roll_slider, 0, 1)
        orientation_layout.addWidget(self.roll_value_label, 0, 2)

        orientation_layout.addWidget(QLabel("Pitch (Y):"), 1, 0)
        orientation_layout.addWidget(self.pitch_slider, 1, 1)
        orientation_layout.addWidget(self.pitch_value_label, 1, 2)

        orientation_layout.addWidget(QLabel("Yaw (Z):"), 2, 0)
        orientation_layout.addWidget(self.yaw_slider, 2, 1)
        orientation_layout.addWidget(self.yaw_value_label, 2, 2)

        # Add actual orientation labels
        orientation_layout.addWidget(QLabel("Actual Roll:"), 0, 3)
        self.actual_roll_label = QLabel("0.00°")
        orientation_layout.addWidget(self.actual_roll_label, 0, 4)

        orientation_layout.addWidget(QLabel("Actual Pitch:"), 1, 3)
        self.actual_pitch_label = QLabel("0.00°")
        orientation_layout.addWidget(self.actual_pitch_label, 1, 4)

        orientation_layout.addWidget(QLabel("Actual Yaw:"), 2, 3)
        self.actual_yaw_label = QLabel("0.00°")
        orientation_layout.addWidget(self.actual_yaw_label, 2, 4)

        orientation_group.setLayout(orientation_layout)
        layout.addWidget(orientation_group)

        return widget

    def create_trajectory_widget(self) -> QWidget:
        """Create a widget for predefined trajectory control."""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setSpacing(20)

        # Create a trajectory selection group
        trajectory_group = QGroupBox("Predefined Trajectories")
        trajectory_layout = QVBoxLayout()

        # Create a combo box to select a trajectory type
        type_layout = QHBoxLayout()
        self.trajectory_type_label = QLabel("Trajectory type:")
        self.trajectory_type_combo = QComboBox()
        self.trajectory_type_combo.addItem("Circle (XY plane)")
        self.trajectory_type_combo.addItem("Square (XY plane)")
        type_layout.addWidget(self.trajectory_type_label)
        type_layout.addWidget(self.trajectory_type_combo)

        # Create trajectory parameters group
        params_group = QGroupBox("Trajectory Parameters")
        params_layout = QGridLayout()

        # Duration slider
        duration_label = QLabel("Duration (seconds):")
        self.duration_spinbox = QSpinBox()
        self.duration_spinbox.setRange(1, 10)
        self.duration_spinbox.setValue(5)
        self.duration_spinbox.setSingleStep(1)

        # Size slider
        size_label = QLabel("Size (meters):")
        self.size_spinbox = QDoubleSpinBox()
        self.size_spinbox.setRange(0.05, 0.3)
        self.size_spinbox.setValue(0.1)
        self.size_spinbox.setSingleStep(0.01)
        self.size_spinbox.setDecimals(2)

        params_layout.addWidget(duration_label, 0, 0)
        params_layout.addWidget(self.duration_spinbox, 0, 1)
        params_layout.addWidget(size_label, 1, 0)
        params_layout.addWidget(self.size_spinbox, 1, 1)
        params_group.setLayout(params_layout)

        # Create a button to send the trajectory
        self.send_trajectory_button = QPushButton("Send Trajectory")
        self.send_trajectory_button.clicked.connect(self.send_trajectory)

        # Add status label for trajectory
        self.trajectory_status_label = QLabel("Ready to send trajectory")

        # Add widgets to the layout
        trajectory_layout.addLayout(type_layout)
        trajectory_layout.addWidget(params_group)
        trajectory_layout.addWidget(self.send_trajectory_button)
        trajectory_layout.addWidget(self.trajectory_status_label)

        trajectory_group.setLayout(trajectory_layout)
        layout.addWidget(trajectory_group)

        return widget

    def create_double_spinbox(self, min_val, max_val, step, default_val):
        spinbox = QDoubleSpinBox()
        spinbox.setRange(min_val, max_val)
        spinbox.setSingleStep(step)
        spinbox.setValue(default_val)
        spinbox.setDecimals(4)
        return spinbox

    def create_position_slider(self, axis, default_value, min_val, max_val):
        """Create a slider for position input with specified range."""
        slider = QSlider(Qt.Horizontal)

        # We'll use 1000 steps for better precision
        slider.setRange(0, 1000)

        # Calculate the slider value based on min-max range
        normalized_value = (default_value - min_val) / (max_val - min_val)
        slider_value = int(normalized_value * 1000)
        slider.setValue(slider_value)

        # Store the range with the slider as properties
        slider.setProperty("min_val", min_val)
        slider.setProperty("max_val", max_val)
        slider.setProperty("axis", axis)

        slider.setTickPosition(QSlider.TicksBelow)
        slider.setTickInterval(100)  # 10 ticks along the slider

        # Set a fixed width to make sliders smaller
        slider.setFixedWidth(250)

        return slider

    def update_position_label(self, axis, slider_value, label):
        """Update position value label based on slider position."""
        min_val = self.position_ranges[axis]["min"]
        max_val = self.position_ranges[axis]["max"]

        # Convert slider value (0-1000) to actual position value
        normalized = slider_value / 1000.0
        actual_value = min_val + normalized * (max_val - min_val)

        # Update the label with 4 decimal places
        label.setText(f"{actual_value:.4f}")

    def get_position_value(self, slider):
        """Get the actual position value from slider."""
        # Get the properties from the slider
        min_val = slider.property("min_val")
        max_val = slider.property("max_val")

        # Convert slider value (0-1000) to actual position value
        normalized = slider.value() / 1000.0
        actual_value = min_val + normalized * (max_val - min_val)

        return actual_value

    def create_angle_slider(self, angle_value):
        """Create a slider for angle input from -180 to 180 degrees."""
        slider = QSlider(Qt.Horizontal)
        slider.setRange(-180, 180)
        slider.setValue(int(angle_value))
        slider.setTickPosition(QSlider.TicksBelow)
        slider.setTickInterval(30)  # Tick marks every 30 degrees

        # Set a fixed width to make sliders smaller
        slider.setFixedWidth(250)

        return slider

    def update_angle_label(self, value, label):
        """Update label to show current slider value in degrees."""
        label.setText(f"{value:.1f}°")

    def get_slider_angle(self, slider):
        """Get the angle value from a slider."""
        return float(slider.value())

    def toggle_realtime_updates(self, state: int) -> None:
        """Toggle the real-time update mode."""
        self.realtime_updates_enabled = state == Qt.Checked
        if self.realtime_updates_enabled:
            self.status_label.setText("Real-time updates enabled")
        else:
            self.status_label.setText(
                "Real-time updates disabled - use 'Send Command' button"
            )

    def on_slider_changed(self) -> None:
        """Handle slider value changes to update robot position in real time."""
        if self.realtime_updates_enabled:
            self.send_command(show_log=False)

    def send_command(self, show_log: bool = True) -> None:
        """Send the current position and orientation to the robot."""
        # Create and publish pose command
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "world"

        # Get position values from sliders
        x = self.get_position_value(self.pos_x_slider)
        y = self.get_position_value(self.pos_y_slider)
        z = self.get_position_value(self.pos_z_slider)

        # Set position
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z

        # Get angle values from sliders
        roll = self.get_slider_angle(self.roll_slider)
        pitch = self.get_slider_angle(self.pitch_slider)
        yaw = self.get_slider_angle(self.yaw_slider)

        # Convert Euler angles to quaternion
        qx, qy, qz, qw = self.euler_to_quaternion(roll, pitch, yaw)

        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw

        # Publish the messages
        self.pose_pub.publish(pose_msg)

        # Update status message
        if self.realtime_updates_enabled:
            self.status_label.setText(
                f"Real-time update: Position: ({x:.4f}, {y:.4f}, {z:.4f}), "
                f"Orientation: ({roll:.1f}°, {pitch:.1f}°, {yaw:.1f}°)"
            )
        else:
            self.status_label.setText(
                f"Command sent: Position: ({x:.4f}, {y:.4f}, {z:.4f}), "
                f"Orientation: ({roll:.1f}°, {pitch:.1f}°, {yaw:.1f}°)"
            )

        # Only log if requested (to avoid excessive logs in real-time mode)
        if show_log:
            self.get_logger().info(
                f"Command sent: Position: ({x:.4f}, {y:.4f}, {z:.4f}), "
                f"Orientation: ({roll}°, {pitch}°, {yaw}°)"
            )

    def reset_values(self) -> None:
        """Reset all sliders to the configuration values."""
        # Reset pose values to the loaded configuration
        pos = self.config["position"]
        ori = self.config["orientation"]

        # Reset position sliders
        for axis, slider, range_key in [
            ("x", self.pos_x_slider, "x"),
            ("y", self.pos_y_slider, "y"),
            ("z", self.pos_z_slider, "z"),
        ]:
            # Calculate normalized value for the slider
            min_val = self.position_ranges[range_key]["min"]
            max_val = self.position_ranges[range_key]["max"]
            normalized = (pos[axis] - min_val) / (max_val - min_val)
            slider_value = int(normalized * 1000)
            slider.setValue(slider_value)

        # Reset Euler angles (sliders)
        self.roll_slider.setValue(int(ori["roll"]))
        self.pitch_slider.setValue(int(ori["pitch"]))
        self.yaw_slider.setValue(int(ori["yaw"]))

        self.get_logger().info("Values reset to configuration")

        # If real-time updates are enabled, send the command after reset
        if self.realtime_updates_enabled:
            self.send_command()

    def send_trajectory(self) -> None:
        """Generate and send a CartesianTrajectory message."""
        if self.is_sending_trajectory:
            return

        trajectory_type = self.trajectory_type_combo.currentText()
        duration = self.duration_spinbox.value()
        size = self.size_spinbox.value()

        self.is_sending_trajectory = True

        # Update UI
        self.send_trajectory_button.setEnabled(False)
        self.trajectory_status_label.setText(
            f"Generating trajectory: {trajectory_type}"
        )

        # Get the current position as the center point for trajectories
        pos_x = self.get_position_value(self.pos_x_slider)
        pos_y = self.get_position_value(self.pos_y_slider)
        pos_z = self.get_position_value(self.pos_z_slider)

        # Get current orientation
        roll = self.get_slider_angle(self.roll_slider)
        pitch = self.get_slider_angle(self.pitch_slider)
        yaw = self.get_slider_angle(self.yaw_slider)

        # Create a CartesianTrajectory message
        trajectory = CartesianTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.header.frame_id = "world"

        # Center position and orientation
        start_position = (pos_x, pos_y, pos_z)
        orientation = (roll, pitch, yaw)

        # Generate trajectory points based on type
        if "Circle" in trajectory_type:
            trajectory.points = self.generate_circle_trajectory(
                start_position, orientation, size, duration, self.num_points
            )
        elif "Square" in trajectory_type:
            trajectory.points = self.generate_square_trajectory(
                start_position, orientation, size, duration, self.num_points
            )

        start_p = trajectory.points[0].point.pose.position
        self.get_logger().info(
            f"Start trajectory position: {start_p.x:.4f}, {start_p.y:.4f}, {start_p.z:.4f}"
        )

        # Publish the trajectory
        self.trajectory_pub.publish(trajectory)

        # Update UI
        self.trajectory_status_label.setText(
            f"Trajectory sent: {trajectory_type} ({len(trajectory.points)} points over {duration}s)"
        )
        self.is_sending_trajectory = False
        self.send_trajectory_button.setEnabled(True)

        self.get_logger().info(
            f"Sent {trajectory_type} trajectory with {len(trajectory.points)} points"
        )

    def generate_circle_trajectory(
        self,
        start_position: tuple[float, float, float],
        orientation: tuple[float, float, float],
        radius: float,
        duration: float,
        num_points: int,
    ) -> list[CartesianTrajectoryPoint]:
        """Generate a circular trajectory in the XY plane."""
        pos_x, pos_y, pos_z = start_position
        roll, pitch, yaw = orientation

        # Calculate center position
        centers = [
            (pos_x - radius, pos_y, 0),
            (pos_x + radius, pos_y, np.pi),
            (pos_x, pos_y - radius, np.pi / 2),
            (pos_x, pos_y + radius, -np.pi / 2),
        ]
        centers.sort(key=lambda center: math.sqrt(center[0] ** 2 + center[1] ** 2))
        center_x, center_y, phi_0 = centers[0]

        # Calculate time step and angular velocity
        time_step = duration / num_points
        angular_velocity = 2 * math.pi / duration  # rad/s for complete circle

        trajectory_points = []
        for i in range(num_points):
            t = i * time_step
            angle = angular_velocity * t

            # Position
            x = center_x + radius * math.cos(angle + phi_0)
            y = center_y + radius * math.sin(angle + phi_0)
            z = pos_z

            # Velocity (derivative of position)
            vx = -radius * angular_velocity * math.sin(angle + phi_0)
            vy = radius * angular_velocity * math.cos(angle + phi_0)
            vz = 0.0

            # Acceleration (derivative of velocity)
            ax = -radius * angular_velocity * angular_velocity * math.cos(angle + phi_0)
            ay = -radius * angular_velocity * angular_velocity * math.sin(angle + phi_0)
            az = 0.0

            # Add point to trajectory
            point = self.create_trajectory_point(
                x=x,
                y=y,
                z=z,
                roll=roll,
                pitch=pitch,
                yaw=yaw,
                vx=vx,
                vy=vy,
                vz=vz,
                wx=0.0,
                wy=0.0,
                wz=0.0,  # Linear and angular velocity
                ax=ax,
                ay=ay,
                az=az,
                awx=0.0,
                awy=0.0,
                awz=0.0,  # Linear and angular acceleration
                time_from_start=t,
            )
            trajectory_points.append(point)

        return trajectory_points

    def generate_square_trajectory(
        self,
        start_position: tuple[float, float, float],
        orientation: tuple[float, float, float],
        size: float,
        duration: float,
        num_points: int,
    ) -> list[CartesianTrajectoryPoint]:
        """Generate a square trajectory in the XY plane."""
        pos_x, pos_y, pos_z = start_position
        roll, pitch, yaw = orientation
        shifts = [(-1, 0), (0, -1), (1, 0), (0, 1)]

        # Calculate center position
        centers = [
            (pos_x - size, pos_y - size, 0),
            (pos_x + size, pos_y - size, 1),
            (pos_x + size, pos_y + size, 2),
            (pos_x - size, pos_y + size, 3),
        ]
        centers.sort(key=lambda center: math.sqrt(center[0] ** 2 + center[1] ** 2))
        _, _, idx = centers[0]

        # Calculate the time step
        time_step = duration / num_points

        # Define the four corners of the square, relative to center
        shifts = [shifts[(i + idx) % 4] for i in range(4)]
        corners = [(pos_x, pos_y)]
        for m_x, m_y in shifts:
            cur_x, cur_y = corners[-1]
            corners.append((cur_x + m_x * size, cur_y + m_y * size))

        # Generate points along each edge
        points_per_side = num_points // 4
        duration_per_side = duration / 4
        trajectory_points = []
        count = 0

        for i in range(4):
            start_x, start_y = corners[i]
            end_x, end_y = corners[(i + 1) % 4]

            # Compute direction vector for this edge
            dx = end_x - start_x
            dy = end_y - start_y

            # Velocity vector (constant along edge)
            max_vx = 8 * dx / duration
            max_vy = 8 * dy / duration

            for j in range(points_per_side):
                t = j / points_per_side
                x = start_x + t * (end_x - start_x)
                y = start_y + t * (end_y - start_y)
                z = pos_z
                t = t * time_step

                if t < duration_per_side / 2:
                    vx = (2 * max_vx * t) / duration_per_side
                    vy = (2 * max_vy * t) / duration_per_side
                    ax = 2 * max_vx / duration_per_side
                    ay = 2 * max_vy / duration_per_side
                else:
                    vx = -(2 * max_vx * t) / duration_per_side + 2 * max_vx
                    vy = -(2 * max_vy * t) / duration_per_side + 2 * max_vy
                    ax = -2 * max_vx / duration_per_side
                    ay = -2 * max_vy / duration_per_side

                # Zero acceleration for constant velocity along edges
                # At corners, we'd need infinite acceleration, but we'll use zero as an approximation
                point = self.create_trajectory_point(
                    x=x,
                    y=y,
                    z=z,
                    roll=roll,
                    pitch=pitch,
                    yaw=yaw,
                    vx=vx,
                    vy=vy,
                    vz=0.0,
                    wx=0.0,
                    wy=0.0,
                    wz=0.0,
                    ax=ax,
                    ay=ay,
                    az=0.0,
                    awx=0.0,
                    awy=0.0,
                    awz=0.0,
                    time_from_start=count * time_step,
                )
                trajectory_points.append(point)
                count += 1

        return trajectory_points

    def create_trajectory_point(
        self,
        x: float,
        y: float,
        z: float,
        roll: float,
        pitch: float,
        yaw: float,
        vx: float = 0.0,
        vy: float = 0.0,
        vz: float = 0.0,
        wx: float = 0.0,
        wy: float = 0.0,
        wz: float = 0.0,
        ax: float = 0.0,
        ay: float = 0.0,
        az: float = 0.0,
        awx: float = 0.0,
        awy: float = 0.0,
        awz: float = 0.0,
        time_from_start: float = 0.0,
    ) -> CartesianTrajectoryPoint:
        """Create a CartesianTrajectoryPoint with the given parameters."""
        point = CartesianTrajectoryPoint()

        # Set position
        point.point.pose.position.x = x
        point.point.pose.position.y = y
        point.point.pose.position.z = z

        # Convert Euler angles to quaternion and set orientation
        qx, qy, qz, qw = self.euler_to_quaternion(roll, pitch, yaw)
        point.point.pose.orientation.x = qx
        point.point.pose.orientation.y = qy
        point.point.pose.orientation.z = qz
        point.point.pose.orientation.w = qw

        # Set velocity
        point.point.velocity.linear.x = vx
        point.point.velocity.linear.y = vy
        point.point.velocity.linear.z = vz
        point.point.velocity.angular.x = wx
        point.point.velocity.angular.y = wy
        point.point.velocity.angular.z = wz

        # Set acceleration
        point.point.acceleration.linear.x = ax
        point.point.acceleration.linear.y = ay
        point.point.acceleration.linear.z = az
        point.point.acceleration.angular.x = awx
        point.point.acceleration.angular.y = awy
        point.point.acceleration.angular.z = awz

        # Set time from start
        point.time_from_start.sec = int(time_from_start)
        point.time_from_start.nanosec = int(
            (time_from_start - int(time_from_start)) * 1e9
        )

        return point

    def run(self) -> None:
        """Main execution loop."""
        # Process any pending ROS events
        while self.window.is_running:
            rclpy.spin_once(self, timeout_sec=0.1)


def main(args=None) -> None:
    """Main entry point."""
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    gui = YumiUdwadiaGui(app)

    threading.Thread(target=gui.run).start()
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
