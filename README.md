# YuMi Udwadia Controller

A task space controller for the ABB YuMi robot that implements Udwadia's formulation for constrained dynamics.

## Overview

This package provides a ROS2 controller for the ABB YuMi collaborative robot that enables precise task space control using Udwadia's equation approach. The controller solves a quadratic programming (QP) problem to compute optimal control forces that satisfy task constraints while minimizing a control cost function.

Key features:
- Task space (Cartesian) control using Udwadia's formulation
- Support for both direct pose commands and trajectory execution
- Constraint handling for maintaining end-effector constraints
- Real-time performance using efficient QP solver

## Related Controllers

This package is part of a suite of controllers for the YuMi robot:

- **YumiUdwadiaController**: This controller, which implements Udwadia's formulation
- **YumiInverseDynamicsController**: A more traditional inverse dynamics controller for the YuMi robot that computes joint torques based on desired accelerations

## Dependencies

- ROS2 (tested on Humble)
- [controller_interface](https://github.com/ros-controls/ros2_controllers)
- [Pinocchio](https://github.com/stack-of-tasks/pinocchio) - Rigid body dynamics library
- [ProxSuite](https://github.com/Simple-Robotics/proxsuite) - QP solver
- Eigen3

## Installation

1. Clone this repository into your ROS2 workspace's src directory:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/your-username/yumi-control.git
   ```

2. Install dependencies:
   ```bash
   sudo apt-get install ros-$ROS_DISTRO-controller-interface
   sudo apt-get install ros-$ROS_DISTRO-pinocchio
   ```

3. Build the workspace:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select yumi_control
   ```

## Usage

### Loading the Controller

1. Configure the controller in your `controllers.yaml`:
   ```yaml
   yumi_udwadia_controller:
     type: yumi_control/YumiUdwadiaController
     joints:
       - joint_1
       - joint_2
       # Add all YuMi joints
     kp: 100.0
     kd: 10.0
     start_link: "base_link"
     end_link: "tool0"
   ```

2. Launch with ROS2 control:
   ```bash
   ros2 launch yumi_control yumi_udwadia_controller.launch.py
   ```

### Sending Commands

#### Pose Command
```bash
ros2 topic pub /yumi_udwadia_controller/pose_command geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.4, y: 0.0, z: 0.4}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

#### Trajectory Command
```bash
ros2 topic pub /yumi_udwadia_controller/trajectory_command moveit_msgs/msg/CartesianTrajectory \
  "{header: {frame_id: 'base_link'}, points: [{pose: {...}, velocity: {...}, acceleration: {...}}, ...]}"
```

## Docker Development Environment

This project includes a ready-to-use Docker development environment with all necessary dependencies pre-installed.

### Using Docker Compose

1. Build and start the container using docker-compose:
   ```bash
   docker-compose build
   docker-compose up -d
   ```

2. Access the running container:
   ```bash
   docker exec -it ros-humble bash
   ```

3. Inside the container, build the project:
   ```bash
   cd /home/ros/course-project
   colcon build --packages-select yumi_control
   source install/setup.bash
   ```

4. Run the controller:
   ```bash
   ros2 launch yumi_control yumi_udwadia_controller.launch.py
   ```

### Using Visual Studio Code Dev Containers

This project includes a `.devcontainer` configuration for Visual Studio Code:

1. Install the VS Code [Remote - Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension
2. Open the project folder in VS Code
3. Click on the "Reopen in Container" notification or use the Command Palette (`F1`) and select "Remote-Containers: Reopen in Container"
4. VS Code will build and start the container, then open the project inside it
5. Open a terminal in VS Code and build the project:
   ```bash
   colcon build --packages-select yumi_control
   source install/setup.bash
   ```

6. Run the controller:
   ```bash
   ros2 launch yumi_control yumi_udwadia_controller.launch.py
   ```

The Dev Container includes the following features:
- Full ROS2 Humble development environment
- All dependencies pre-installed (Pinocchio, ProxSuite, ROS2 Controllers)
- X11 forwarding for GUI applications
- C++ and Python development tools

## Controller Description

The YuMi Udwadia controller implements Udwadia's equation for task space control:

```
F = M * (M^-1 * F_c + A^+ * (b - A * M^-1 * F_c))
```

where:
- `F` is the control force
- `M` is the mass matrix
- `F_c` is the bias force (Coriolis, gravity)
- `A` and `b` represent task constraints (`A * qddot = b`)
- `A^+` is the Moore-Penrose pseudoinverse of `A`

The controller uses Pinocchio for computing forward kinematics, Jacobians, and dynamics, and ProxSuite for solving the QP optimization problem to find optimal control forces.

## Parameters

| Parameter | Type | Description | Default |
|-----------|------|-------------|---------|
| `kp` | double | Position gain | 100.0 |
| `kd` | double | Velocity gain | 10.0 |
| `constraint_kp` | double | Constraint position gain | 100.0 |
| `constraint_kd` | double | Constraint velocity gain | 10.0 |
| `control_weight` | double | Weight for control cost in QP | 1.0 |
| `start_link` | string | Start link name for controller | - |
| `end_link` | string | End link name for controller | - |
| `robot_description_topic` | string | Topic for robot description | "robot_description" |

## Topic Interface

### Subscriptions

| Topic | Type | Description |
|-------|------|-------------|
| `~/pose_command` | geometry_msgs/PoseStamped | Target pose for end-effector |
| `~/trajectory_command` | moveit_msgs/CartesianTrajectory | Cartesian trajectory to follow |
| `robot_description` | std_msgs/String | URDF robot description |

### Publications (Debug)

| Topic | Type | Description |
|-------|------|-------------|
| `~/debug/joint_states` | sensor_msgs/JointState | Internal joint states |
| `~/debug/cartesian` | moveit_msgs/CartesianTrajectoryPoint | Current cartesian state |

## License

This project is licensed under the MIT License - see the LICENSE file for details. 