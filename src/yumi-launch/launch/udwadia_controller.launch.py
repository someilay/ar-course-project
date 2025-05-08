#!/usr/bin/env python3

import launch
import launch.event_handlers
import launch.launch_description_sources
import launch.substitutions
import launch_ros
import launch_ros.substitutions
from launch.actions import OpaqueFunction


def generate_launch_description():
    ld = launch.LaunchDescription()

    # Parameters
    yumi_launch_pkg_name = "yumi-launch"
    controller_name_value = "udwadia_controller"
    template_name_value = "udwadia_controller.yaml.tmp"

    # Arguments values
    ns = launch.substitutions.LaunchConfiguration("namespace")
    use_pose_publisher = launch.substitutions.LaunchConfiguration("use_pose_publisher")
    use_trajectory_publisher = launch.substitutions.LaunchConfiguration(
        "use_trajectory_publisher"
    )
    use_gui = launch.substitutions.LaunchConfiguration("use_gui")

    # Topic configurations for GUI
    target_pose_topic = launch.substitutions.LaunchConfiguration("target_pose_topic")

    # Paths
    yumi_launch_pkg_path = launch_ros.substitutions.FindPackageShare(
        yumi_launch_pkg_name
    )
    base_launch_file = launch.substitutions.PathJoinSubstitution(
        [yumi_launch_pkg_path, "launch", "base.launch.py"]
    )
    target_pose_topic = launch.substitutions.PathJoinSubstitution(
        [ns, controller_name_value, "command"]
    )
    debug_cartesian_topic = launch.substitutions.PathJoinSubstitution(
        [ns, controller_name_value, "debug", "cartesian"]
    )

    # Define arguments
    ld.add_action(
        launch.actions.DeclareLaunchArgument(
            "namespace",
            default_value="yumi",
            description="Namespace for the robot",
        ),
    )
    ld.add_action(
        launch.actions.DeclareLaunchArgument(
            "use_pose_publisher",
            default_value="false",
            description="Start the pose publisher that sends continuous pose commands",
        ),
    )
    ld.add_action(
        launch.actions.DeclareLaunchArgument(
            "use_trajectory_publisher",
            default_value="false",
            description="Start the trajectory publisher that sends a sample trajectory",
        ),
    )
    ld.add_action(
        launch.actions.DeclareLaunchArgument(
            "use_gui",
            default_value="true",
            description="Start the GUI for sending commands to the controller",
        ),
    )

    # Include base launch file
    include_base_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            base_launch_file
        ),
        launch_arguments={
            "namespace": ns,
            "controller_name": controller_name_value,
            "template_name": template_name_value,
        }.items(),
    )

    # Add the pose publisher node (conditional)
    pose_publisher_node = launch_ros.actions.Node(
        package="yumi-launch",
        executable="publish_pose_command.py",
        name="pose_publisher",
        namespace=ns,
        parameters=[
            {"namespace": ns},
            {"controller_name": controller_name_value},
            {"publish_rate": 10.0},
        ],
        output="screen",
        condition=launch.conditions.IfCondition(use_pose_publisher),
    )

    # Add the trajectory publisher node (conditional)
    trajectory_publisher_node = launch_ros.actions.Node(
        package="yumi-launch",
        executable="publish_cartesian_trajectory.py",
        name="trajectory_publisher",
        namespace=ns,
        parameters=[{"namespace": ns}, {"controller_name": controller_name_value}],
        output="screen",
        condition=launch.conditions.IfCondition(use_trajectory_publisher),
    )

    # Add the GUI node (conditional)
    gui_node = launch_ros.actions.Node(
        package="yumi-udwadia-gui",
        executable="yumi_gui",
        name="yumi_udwadia_gui",
        parameters=[
            {
                "target_pose_topic": target_pose_topic,
                "debug_cartesian_topic": debug_cartesian_topic,
            }
        ],
        output="screen",
        condition=launch.conditions.IfCondition(use_gui),
    )

    # Add the publisher nodes and base launch
    ld.add_action(pose_publisher_node)
    ld.add_action(trajectory_publisher_node)
    ld.add_action(gui_node)
    ld.add_action(include_base_launch)

    return ld
