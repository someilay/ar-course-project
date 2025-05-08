import sys
import rclpy
import threading
import numpy as np
import signal
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped, AccelStamped
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
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QDoubleSpinBox, QPushButton, QTabWidget, QGridLayout, QGroupBox
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

        # Declare parameters for topic names
        self.declare_parameter("target_pose_topic")

        # Get parameters
        target_pose_topic = (
            self.get_parameter("target_pose_topic").get_parameter_value().string_value
        )

        # Log the topic names
        self.get_logger().info(f"Target pose topic: {target_pose_topic}")

        # Create publishers for pose, velocity, and acceleration commands
        self.pose_pub = self.create_publisher(PoseStamped, target_pose_topic, 10)

        # Initialize the GUI
        self.app = app
        self.init_gui()

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

        # Position inputs (x, y, z)
        self.pos_x = self.create_double_spinbox(-2.0, 2.0, 0.001, 0.6)
        self.pos_y = self.create_double_spinbox(-2.0, 2.0, 0.001, -0.15)
        self.pos_z = self.create_double_spinbox(-2.0, 2.0, 0.001, 0.5)

        position_layout.addWidget(QLabel("X:"), 0, 0)
        position_layout.addWidget(self.pos_x, 0, 1)
        position_layout.addWidget(QLabel("Y:"), 1, 0)
        position_layout.addWidget(self.pos_y, 1, 1)
        position_layout.addWidget(QLabel("Z:"), 2, 0)
        position_layout.addWidget(self.pos_z, 2, 1)

        position_group.setLayout(position_layout)
        layout.addWidget(position_group)

        # Create orientation group
        orientation_group = QGroupBox("Orientation (Quaternion)")
        orientation_layout = QGridLayout()

        # Orientation inputs (quaternion x, y, z, w)
        self.quat_x = self.create_double_spinbox(-1.0, 1.0, 0.001, 0.0)
        self.quat_y = self.create_double_spinbox(-1.0, 1.0, 0.001, 0.0)
        self.quat_z = self.create_double_spinbox(-1.0, 1.0, 0.001, 0.0)
        self.quat_w = self.create_double_spinbox(-1.0, 1.0, 0.001, 1.0)

        orientation_layout.addWidget(QLabel("X:"), 0, 0)
        orientation_layout.addWidget(self.quat_x, 0, 1)
        orientation_layout.addWidget(QLabel("Y:"), 1, 0)
        orientation_layout.addWidget(self.quat_y, 1, 1)
        orientation_layout.addWidget(QLabel("Z:"), 2, 0)
        orientation_layout.addWidget(self.quat_z, 2, 1)
        orientation_layout.addWidget(QLabel("W:"), 3, 0)
        orientation_layout.addWidget(self.quat_w, 3, 1)

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
        arr = np.array([self.quat_x.value(), self.quat_y.value(), self.quat_z.value(), self.quat_w.value()])

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
        # Reset pose values
        self.pos_x.setValue(0.6)
        self.pos_y.setValue(-0.2)
        self.pos_z.setValue(0.5)
        self.quat_x.setValue(0.0)
        self.quat_y.setValue(0.0)
        self.quat_z.setValue(0.0)
        self.quat_w.setValue(1.0)

        self.get_logger().info("Values reset")

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
